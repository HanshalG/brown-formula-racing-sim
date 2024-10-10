import numpy as np
import cv2
import csv
import matplotlib.pyplot as plt
from scipy.signal import savgol_filter

def process_image(image_path):
    # Read image with alpha channel
    img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    
    # Check if the image has an alpha channel
    if img.shape[2] == 4:
        # Split the image into color channels and alpha channel
        b, g, r, alpha = cv2.split(img)
        # Create a binary image from the alpha channel
        _, binary = cv2.threshold(alpha, 250, 255, cv2.THRESH_BINARY)
    else:
        # If no alpha channel, convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # Threshold the grayscale image
        _, binary = cv2.threshold(gray, 250, 255, cv2.THRESH_BINARY)
    
    # Ensure binary is in the correct format
    binary = binary.astype(np.uint8)
    
    # Find contours
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    
    # Assume the largest contour is our track
    if contours:
        track_contour = max(contours, key=cv2.contourArea)
    else:
        raise ValueError("No contours found in the image.")
    
    return track_contour

def segment_track(contour):
    x, y = contour[:, 0, 0], contour[:, 0, 1]
    
    # Increase smoothing
    window_length = min(len(x) // 5, 101)
    if window_length % 2 == 0:
        window_length += 1
    x_smooth = savgol_filter(x, window_length, 3)
    y_smooth = savgol_filter(y, window_length, 3)
    
    dx = np.gradient(x_smooth)
    dy = np.gradient(y_smooth)
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    curvature = (dx*ddy - dy*ddx) / (dx**2 + dy**2)**1.5
    
    segments = []
    current_direction = "Straight"
    section_start = 0
    curvature_threshold = 0.005
    min_segment_length = 10
    
    for i, c in enumerate(curvature):
        if c > curvature_threshold and (current_direction != "Left" or i - section_start > min_segment_length):
            if i > section_start:
                segments.append((current_direction, section_start, i))
            current_direction = "Left"
            section_start = i
        elif c < -curvature_threshold and (current_direction != "Right" or i - section_start > min_segment_length):
            if i > section_start:
                segments.append((current_direction, section_start, i))
            current_direction = "Right"
            section_start = i
        elif abs(c) <= curvature_threshold and (current_direction != "Straight" or i - section_start > min_segment_length):
            if i > section_start:
                segments.append((current_direction, section_start, i))
            current_direction = "Straight"
            section_start = i
    
    segments.append((current_direction, section_start, len(curvature)))
    
    processed_segments = []
    for direction, start, end in segments:
        length = np.sum(np.sqrt(np.diff(x_smooth[start:end])**2 + np.diff(y_smooth[start:end])**2))
        if direction != "Straight":
            avg_curvature = np.mean(np.abs(curvature[start:end]))
            radius = 1 / avg_curvature if avg_curvature != 0 else 0
        else:
            radius = 0
        processed_segments.append((direction, length, radius))
    
    return processed_segments

def save_to_csv(segments, output_file):
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["Type", "Length", "Corner Radius"])
        for segment in segments:
            writer.writerow(segment)

def visualize_track(image_path, csv_file):
    img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    segments = []
    with open(csv_file, 'r') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header
        segments = list(reader)
    
    plt.figure(figsize=(12, 8))
    
    # Handle transparent background
    if img.shape[2] == 4:
        # Create a white background
        background = np.ones((img.shape[0], img.shape[1], 3), dtype=np.uint8) * 255
        # Extract RGB channels
        rgb = img[:,:,:3]
        # Extract alpha channel
        alpha = img[:,:,3] / 255.0
        # Blend the image
        blended = (1 - alpha[:,:,np.newaxis]) * background + alpha[:,:,np.newaxis] * rgb
        plt.imshow(blended.astype(np.uint8))
    else:
        plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    
    total_length = sum(float(segment[1]) for segment in segments)
    current_length = 0
    height, width = img.shape[:2]
    for i, segment in enumerate(segments):
        seg_type, length, radius = segment
        length, radius = float(length), float(radius)
        
        pos = current_length / total_length
        x = int(width * 0.1)
        y = int(height * (0.1 + pos * 0.8))
        
        color = 'red' if seg_type == 'Left' else 'blue' if seg_type == 'Right' else 'green'
        
        plt.text(x, y, f"{seg_type}: {length:.2f}, R={radius:.2f}", color=color, fontsize=8)
        
        current_length += length
    
    plt.title("Track Segmentation Visualization")
    plt.axis('off')
    plt.tight_layout()
    plt.show()

def main():
    image_path = "image3.png"  # Replace with your image path
    output_csv = "track_data.csv"
    
    try:
        contour = process_image(image_path)
        segments = segment_track(contour)
        save_to_csv(segments, output_csv)
        visualize_track(image_path, output_csv)
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    main()
