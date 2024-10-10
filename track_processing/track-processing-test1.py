import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
from scipy.signal import savgol_filter
import csv

# Function to load and process the image into a binary format and extract contours
def process_image(image_path):
    img = cv2.imread(image_path)
    if img is None:
        raise FileNotFoundError(f"Unable to read image at {image_path}")
    
    # Convert to grayscale and binary image
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY_INV)
    
    # Extract contours (edges of the track)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        raise ValueError("No contours found in the image")
    
    # Select the largest contour as the track
    track_contour = max(contours, key=cv2.contourArea)
    return img, track_contour

# Function to smooth the contour using spline interpolation
def smooth_contour(contour, smoothness=0.1, resolution=1000):
    contour = contour.squeeze().astype(float)  # Remove unnecessary dimensions
    x, y = contour[:, 0], contour[:, 1]
    
    # Spline interpolation to smooth contour
    tck, u = splprep([x, y], s=smoothness, per=True)
    u_new = np.linspace(u.min(), u.max(), resolution)  # Control resolution here
    x_new, y_new = splev(u_new, tck)
    
    return np.column_stack((x_new, y_new))

# Function to calculate curvature from the x, y coordinates
def calculate_curvature(x, y):
    dx = np.gradient(x)
    dy = np.gradient(y)
    d2x = np.gradient(dx)
    d2y = np.gradient(dy)
    
    # Curvature formula
    curvature = np.abs(dx * d2y - dy * d2x) / (dx**2 + dy**2)**1.5
    return curvature

# Function to segment track into straight and curved sections based on curvature
def segment_track(smoothed_contour, curvature_threshold=0.0005, straight_threshold=0.0001):
    x, y = smoothed_contour[:, 0], smoothed_contour[:, 1]
    curvature = calculate_curvature(x, y)
    
    # Dynamic window length for smoothing, depends on track length
    window_length = max(51, len(curvature) // 20 | 1)  # Ensure odd length for filter
    curvature_smooth = savgol_filter(curvature, window_length=window_length, polyorder=3)
    
    segments = []
    segment_start = 0
    current_type = "Straight" if curvature_smooth[0] < straight_threshold else "Turn"
    
    for i in range(1, len(smoothed_contour)):
        # Identify changes from straight to turn or turn to straight based on thresholds
        if (current_type == "Straight" and curvature_smooth[i] > straight_threshold) or \
           (current_type == "Turn" and abs(curvature_smooth[i] - curvature_smooth[segment_start]) > curvature_threshold):
            
            # Capture segment properties
            segment_points = smoothed_contour[segment_start:i]
            avg_curvature = np.mean(curvature_smooth[segment_start:i])
            radius = 1 / avg_curvature if avg_curvature > 0 else float('inf')
            length = np.sum(np.sqrt(np.diff(segment_points[:, 0])**2 + np.diff(segment_points[:, 1])**2))
            
            segments.append({
                "start": tuple(segment_points[0]),
                "end": tuple(segment_points[-1]),
                "points": segment_points,
                "radius": radius,
                "length": length,
                "curvature": avg_curvature,
                "type": current_type
            })
            
            # Reset for new segment
            segment_start = i
            current_type = "Turn" if current_type == "Straight" else "Straight"
    
    # Add final segment
    segment_points = smoothed_contour[segment_start:]
    avg_curvature = np.mean(curvature_smooth[segment_start:])
    radius = 1 / avg_curvature if avg_curvature > 0 else float('inf')
    length = np.sum(np.sqrt(np.diff(segment_points[:, 0])**2 + np.diff(segment_points[:, 1])**2))
    segments.append({
        "start": tuple(segment_points[0]),
        "end": tuple(segment_points[-1]),
        "points": segment_points,
        "radius": radius,
        "length": length,
        "curvature": avg_curvature,
        "type": current_type
    })
    
    return segments

# Function to visualize the segmented track with matplotlib
def plot_track(img, segments):
    plt.figure(figsize=(12, 8))
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    
    # Colors for straight and turn segments
    colors = {'Straight': 'green', 'Turn': 'red'}
    
    for segment in segments:
        color = colors[segment['type']]
        plt.plot(segment["points"][:, 0], segment["points"][:, 1], color=color, linewidth=2)
        
        # Annotate the segments with their radius and length
        mid_point = segment["points"][len(segment["points"])//2]
        #plt.annotate(f'R={segment["radius"]/100:.1f}m\nL={segment["length"]/100:.1f}m', xy=mid_point, xytext=(10, 10), 
        #             textcoords='offset points', ha='left', va='bottom',
        #             bbox=dict(boxstyle='round,pad=0.5', fc='yellow', alpha=0.5),
        #             arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))
    
    plt.title("Track Represented as Turns and Straights")
    plt.axis('off')
    plt.show()

# Save track data to a CSV file for further analysis
def save_to_csv(segments, filename="track_data_turns.csv"):
    with open(filename, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Segment Number", "Type", "Length (m)", "Radius (m)"])
        for i, segment in enumerate(segments, 1):
            length = segment['length'] / 100  # Convert pixels to meters (assuming 100 pixels = 1 meter)
            radius = segment['radius'] / 100  # Convert pixels to meters
            
            writer.writerow([i, segment['type'], f"{length:.2f}", f"{radius:.2f}"])
    
    print(f"CSV file '{filename}' has been created with {len(segments)} segments.")

# Main function to run the processing pipeline
def main():
    image_path = "image.png"  # Path to the image file
    
    try:
        # Step 1: Process the image and extract contour
        img, contour = process_image(image_path)
        
        # Step 2: Smooth the contour (increase resolution for finer segmenting)
        smoothed_contour = smooth_contour(contour, smoothness=0.001, resolution=3000)
        
        # Step 3: Segment the track into turns and straights
        segments = segment_track(smoothed_contour, curvature_threshold=0.05, straight_threshold=0.01)
        
        # Step 4: Visualize the segmented track
        plot_track(img, segments)
        
        # Step 5: Save segments to a CSV file
        save_to_csv(segments)
        
        # Print a summary of the segments
        print(f"Total segments: {len(segments)}")
        for i, segment in enumerate(segments, 1):
            print(f"Segment {i}: Type = {segment['type']}, Length = {segment['length']/100:.2f}m, Radius = {segment['radius']/100:.2f}m")
    
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
