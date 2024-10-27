import csv
import numpy as np
import matplotlib.pyplot as plt
from svgpathtools import svg2paths
from shapely.geometry import Point

# Helper function to convert complex numbers to Shapely Point
def complex_to_point(c):
    return Point(c.real, c.imag)

# Function to scale SVG path points
def scale_path(path, scale_factor):
    # Scale each segment of the path
    scaled_path = []
    for segment in path:
        scaled_segment = [(p.real * scale_factor + p.imag * 1j * scale_factor) for p in segment]
        scaled_path.append(scaled_segment)
    return scaled_path

# Function to calculate radius of curvature given a segment
def calculate_radius_of_curvature(segment):
    points = [complex_to_point(p) for p in segment]
    if len(points) < 3:
        return 0  # Zero radius for straight lines

    # Approximate first and second derivatives using finite differences
    p1, p2, p3 = points[:3]
    x1, y1 = p1.x, p1.y
    x2, y2 = p2.x, p2.y
    x3, y3 = p3.x, p3.y

    # Derivatives using finite differences
    dx1, dy1 = x2 - x1, y2 - y1
    dx2, dy2 = x3 - x2, y3 - y2

    # Curvature formula
    curvature = abs(dx1 * dy2 - dy1 * dx2) / (np.hypot(dx1, dy1) ** 3)
    if curvature == 0:
        return 0  # Zero radius for straight lines

    return 1 / curvature

# Function to classify turn type (left or right)
def classify_turn(segment):
    points = [complex_to_point(p) for p in segment]
    if len(points) < 3:
        return "straight"

    p1, p2, p3 = points[:3]
    vector1 = (p2.x - p1.x, p2.y - p1.y)
    vector2 = (p3.x - p2.x, p3.y - p2.y)

    cross_product = vector1[0] * vector2[1] - vector1[1] * vector2[0]
    return "left" if cross_product > 0 else "right"

# Function to process each segment and output to CSV
def process_segments_to_csv(svg_file, output_csv, scale_factor):
    paths, _ = svg2paths(svg_file)
    segments_data = []

    with open(output_csv, 'w', newline='') as csvfile:
        fieldnames = ['turn_type', 'arc_length', 'turn_radius']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for path in paths:
            scaled_path = scale_path(path, scale_factor)
            for segment in scaled_path:
                arc_length = segment.length() * scale_factor  # Convert length using scale factor
                radius = calculate_radius_of_curvature(segment)
                turn_type = classify_turn(segment)

                segments_data.append((segment, turn_type, arc_length, radius))
                
                writer.writerow({
                    'turn_type': turn_type,
                    'arc_length': arc_length,
                    'turn_radius': radius
                })

    return paths, segments_data

# Function to plot the original track
def plot_original_track(paths, scale_factor):
    fig, ax = plt.subplots()
    for path in paths:
        scaled_path = scale_path(path, scale_factor)
        x = [point.real for segment in scaled_path for point in segment]
        y = [point.imag for segment in scaled_path for point in segment]
        ax.plot(x, y, 'k--', label='Original Spline')
    ax.set_title('Original Track')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.legend(loc='upper right')
    ax.axis('equal')
    plt.show()

# Function to plot the segmented track with segment boundaries
def plot_segmented_track(segments_data):
    fig, ax = plt.subplots()

    # Plot the segmented track
    for segment, turn_type, _, _ in segments_data:
        points = [complex_to_point(p) for p in segment]
        x = [point.x for point in points]
        y = [point.y for point in points]

        # Define color based on turn type
        color = 'b' if turn_type == "left" else 'r' if turn_type == "right" else 'g'
        ax.plot(x, y, color + '-', label=turn_type.capitalize() if turn_type.capitalize() not in ax.get_legend_handles_labels()[1] else "")
        
        # Mark the end point of each segment to visualize boundaries
        ax.plot(x[-1], y[-1], 'ko')  # Black dot at the end of each segment

    ax.set_title('Segmented Track with Turn Types')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.legend(loc='upper right')
    ax.axis('equal')
    plt.show()

# Example usage
scale_factor = 0.01  # Define scale factor (e.g., 1 SVG unit = 0.01 meters)
paths, segments_data = process_segments_to_csv('track.svg', 'output.csv', scale_factor)
plot_original_track(paths, scale_factor)
plot_segmented_track(segments_data)
