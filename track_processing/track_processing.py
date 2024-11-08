import numpy as np
from svgpathtools import svg2paths
from scipy.interpolate import splprep, splev
import csv
import matplotlib.pyplot as plt
from scipy.spatial import distance_matrix

def nearest_neighbor_path(points):
    visited = np.zeros(len(points), dtype=bool)
    path = [0]
    visited[0] = True

    for _ in range(1, len(points) - 5):
        last_point = path[-1]
        dist = distance_matrix([points[last_point]], points)[0]
        dist[visited] = np.inf
        next_point = np.argmin(dist)
        path.append(next_point)
        visited[next_point] = True

    return path

def arc_length(x1, y1, x2, y2):
    return np.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def curvature(dx, dy, ddx, ddy):
    num = dx * ddy - dy * ddx
    denom = (dx**2 + dy**2)**(3/2)
    return num / denom if denom != 0 else np.inf

def direction_change(theta1, theta2, thresh):
    delta_theta = np.mod(theta2 - theta1 + np.pi, 2 * np.pi) - np.pi
    if delta_theta < -thresh:
        return "Right"
    elif delta_theta > thresh:
        return "Left"
    else:
        return "Straight"

def calculate_spline_length(x, y):
    """Calculate total length of the spline using all points"""
    total_length = 0
    for i in range(1, len(x)):
        total_length += arc_length(x[i-1], y[i-1], x[i], y[i])
    return total_length

def fit_spline(x, y, n_segments, threshold=0.03, smoothing=1e-4):
    tck, u = splprep([x, y], s=smoothing)
    u_fine = np.linspace(0, 1, n_segments)
    x_fine, y_fine = splev(u_fine, tck)
    dx_fine, dy_fine = splev(u_fine, tck, der=1)
    ddx_fine, ddy_fine = splev(u_fine, tck, der=2)

    arc_lengths = []
    radii = []
    directions = []

    initial_theta = np.arctan2(dy_fine[0], dx_fine[0])
    for i in range(1, n_segments):
        length = arc_length(x_fine[i-1], y_fine[i-1], x_fine[i], y_fine[i])
        arc_lengths.append(length)

        curve_radius = np.inf if curvature(dx_fine[i], dy_fine[i], ddx_fine[i], ddy_fine[i]) == 0 else 1 / np.abs(curvature(dx_fine[i], dy_fine[i], ddx_fine[i], ddy_fine[i]))
        radii.append(curve_radius)

        theta_i = np.arctan2(dy_fine[i], dx_fine[i])
        direction = direction_change(initial_theta, theta_i, threshold)
        directions.append(direction)
        initial_theta = theta_i

    return x_fine, y_fine, arc_lengths, radii, directions

def analyze_track(svg_file, output_csv, scale_factor=1.0, n_segments=1000, threshold=0.03, smoothing=1e-4):
    paths, _ = svg2paths(svg_file)
    points = []
    for path in paths:
        for t in np.linspace(0, 1, n_segments // len(paths)):
            point = path.point(t)
            points.append([point.real * scale_factor, point.imag * scale_factor])

    points = np.array(points)
    x_fine, y_fine, arc_lengths, radii, directions = fit_spline(points[:, 0], points[:, 1], n_segments, threshold, smoothing)

    # Calculate total lengths for comparison
    total_spline_length = calculate_spline_length(x_fine, y_fine)
    total_segment_length = sum(arc_lengths)
    length_difference_percentage = 100 * abs(total_spline_length - total_segment_length) / total_spline_length

    with open(output_csv, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Direction", "Arc Length", "Radius of Curvature"])
        for i in range(n_segments - 1):
            print(arc_lengths[i], radii[i], directions[i])
            if directions[i] == "Straight":
                writer.writerow(["Straight", arc_lengths[i], 0])
            else:
                writer.writerow([directions[i], arc_lengths[i], radii[i]])

        # Add length comparison at the end of the CSV
        writer.writerow([])
        writer.writerow(["Total Spline Length", total_spline_length])
        writer.writerow(["Total Segment Length", total_segment_length])
        writer.writerow(["Length Difference (%)", length_difference_percentage])

    plt.figure(figsize=(12, 8))
    for i in range(n_segments - 1):
        color = 'r' if directions[i] == "Right" else 'g' if directions[i] == "Left" else 'b'
        plt.plot([x_fine[i], x_fine[i+1]], [y_fine[i], y_fine[i+1]], color=color, linewidth=2)
    
    plt.title('Track Analysis Visualization')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.axis('equal')
    plt.grid(True)
    plt.savefig('track_analysis.png')
    plt.show()

    # Print length comparison to console
    print("\nLength Validation:")
    print(f"Total Spline Length: {total_spline_length:.2f}")
    print(f"Total Segment Length: {total_segment_length:.2f}")
    print(f"Difference: {abs(total_spline_length - total_segment_length):.2f} units")
    print(f"Relative Difference: {length_difference_percentage:.2f}%")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description='Analyze race track from SVG file')
    parser.add_argument('--svg_file', help='Input SVG file path')
    parser.add_argument('--output_csv', help='Output CSV file path')
    parser.add_argument('--scale', type=float, default=1.0, help='Scale factor for SVG coordinates')
    parser.add_argument('--segments', type=int, default=1000, help='Number of segments for analysis')
    parser.add_argument('--threshold', type=float, default=0.03, help='Threshold for direction change')
    parser.add_argument('--smoothing', type=float, default=1e-4, help='Smoothing factor for spline')
    args = parser.parse_args()
    analyze_track(args.svg_file, args.output_csv, args.scale, args.segments, args.threshold, args.smoothing)
