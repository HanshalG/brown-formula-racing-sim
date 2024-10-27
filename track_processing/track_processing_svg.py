import csv
import numpy as np
import matplotlib.pyplot as plt
from svgpathtools import svg2paths
from shapely.geometry import Point, LineString
from scipy.integrate import quad

def complex_to_point(c):
    return Point(c.real, c.imag)

def scale_path(path, scale_factor):
    return [[complex(p.real * scale_factor, p.imag * scale_factor) for p in segment] for segment in path]

def bezier_point(t, p0, p1, p2, p3=None):
    if p3 is None:  # Quadratic Bezier
        return (1-t)**2 * p0 + 2*(1-t)*t * p1 + t**2 * p2
    else:  # Cubic Bezier
        return (1-t)**3 * p0 + 3*(1-t)**2*t * p1 + 3*(1-t)*t**2 * p2 + t**3 * p3

def bezier_derivative(t, p0, p1, p2, p3=None):
    if p3 is None:  # Quadratic Bezier
        return 2 * ((1-t) * (p1 - p0) + t * (p2 - p1))
    else:  # Cubic Bezier
        return 3 * ((1-t)**2 * (p1 - p0) + 2*(1-t)*t * (p2 - p1) + t**2 * (p3 - p2))

def bezier_second_derivative(t, p0, p1, p2, p3=None):
    if p3 is None:  # Quadratic Bezier
        return 2 * (p2 - 2*p1 + p0)
    else:  # Cubic Bezier
        return 6 * ((1-t) * (p2 - 2*p1 + p0) + t * (p3 - 2*p2 + p1))

def curvature(t, p0, p1, p2, p3=None):
    d1 = bezier_derivative(t, p0, p1, p2, p3)
    d2 = bezier_second_derivative(t, p0, p1, p2, p3)
    num = d1.real * d2.imag - d1.imag * d2.real
    denom = (d1.real**2 + d1.imag**2)**1.5
    return abs(num / denom) if denom != 0 else 0

def arc_length_integrand(t, p0, p1, p2, p3=None):
    d = bezier_derivative(t, p0, p1, p2, p3)
    return (d.real**2 + d.imag**2)**0.5

def calculate_arc_length(p0, p1, p2, p3=None):
    if p2 is None:  # Straight line
        return abs(p1 - p0)
    return quad(arc_length_integrand, 0, 1, args=(p0, p1, p2, p3))[0]

def classify_turn(p0, p1, p2, p3=None):
    if p2 is None:  # Straight line
        return "Straight"
    if p3 is None:
        d1, d2 = p1 - p0, p2 - p1
    else:
        d1, d2 = p1 - p0, p3 - p2
    cross_product = d1.real * d2.imag - d1.imag * d2.real
    return "Left" if cross_product > 0 else "Right" if cross_product < 0 else "Straight"

def process_segments_to_csv(svg_file, output_csv, scale_factor):
    paths, _ = svg2paths(svg_file)
    
    with open(output_csv, 'w', newline='') as csvfile:
        fieldnames = ['turn_type', 'arc_length', 'turn_radius']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()

        for path in paths:
            scaled_path = scale_path(path, scale_factor)
            for segment in scaled_path:
                p0 = segment[0]
                p1 = segment[1]
                p2 = segment[2] if len(segment) > 2 else None
                p3 = segment[3] if len(segment) > 3 else None

                turn_type = classify_turn(p0, p1, p2, p3)
                arc_length = calculate_arc_length(p0, p1, p2, p3)
                
                if turn_type == "Straight":
                    turn_radius = float('inf')
                else:
                    # Calculate average curvature
                    curvatures = [curvature(t, p0, p1, p2, p3) for t in np.linspace(0, 1, 100)]
                    avg_curvature = np.mean(curvatures)
                    turn_radius = 1 / avg_curvature if avg_curvature != 0 else float('inf')

                writer.writerow({
                    'turn_type': turn_type,
                    'arc_length': arc_length,
                    'turn_radius': turn_radius
                })

    return paths

def plot_original_spline(svg_file, ax, scale_factor):
    paths, _ = svg2paths(svg_file)
    
    for path in paths:
        points = []
        for segment in path:
            points.extend([segment.point(t) for t in np.linspace(0, 1, 100)])
        
        x = [p.real * scale_factor for p in points]
        y = [p.imag * scale_factor for p in points]
        ax.plot(x, y, 'k--', alpha=0.5, linewidth=1, label='Original Spline')

def plot_track_from_csv(csv_file, svg_file, scale_factor):
    # Read the CSV file
    with open(csv_file, 'r') as f:
        reader = csv.DictReader(f)
        data = list(reader)

    # Extract turn_type, arc_length, and turn_radius
    turn_type = [row['turn_type'] for row in data]
    arc_length = np.array([float(row['arc_length']) for row in data])
    turn_radius = np.array([float(row['turn_radius']) for row in data])

    # Calculate cumulative distance
    distance = np.cumsum(arc_length)

    # Calculate X and Y coordinates
    X = np.zeros(len(distance))
    Y = np.zeros(len(distance))
    heading = 0

    for i in range(1, len(distance)):
        if turn_type[i] == 'Straight':
            X[i] = X[i-1] + arc_length[i] * np.cos(heading)
            Y[i] = Y[i-1] + arc_length[i] * np.sin(heading)
        else:
            radius = turn_radius[i]
            angle = arc_length[i] / radius
            if turn_type[i] == 'Left':
                center_x = X[i-1] - radius * np.sin(heading)
                center_y = Y[i-1] + radius * np.cos(heading)
                X[i] = center_x + radius * np.sin(heading + angle)
                Y[i] = center_y - radius * np.cos(heading + angle)
                heading += angle
            else:  # Right turn
                center_x = X[i-1] + radius * np.sin(heading)
                center_y = Y[i-1] - radius * np.cos(heading)
                X[i] = center_x - radius * np.sin(heading - angle)
                Y[i] = center_y + radius * np.cos(heading - angle)
                heading -= angle

    # Plot the track
    fig, ax = plt.subplots(figsize=(12, 8))
    
    # Plot the original spline
    plot_original_spline(svg_file, ax, scale_factor)
    
    # Plot the processed track
    ax.plot(X, Y, 'b-', linewidth=2, label='Processed Track')
    
    plt.title('Track Visualization: Original Spline vs Processed Track')
    plt.xlabel('X coordinate')
    plt.ylabel('Y coordinate')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()


# Example usage
scale_factor = 0.26 # Define scale factor (e.g., 1 SVG unit = 1.48 meters)
paths = process_segments_to_csv('2024_autox.svg', 'output.csv', scale_factor)
plot_track_from_csv('output.csv', '2024_autox.svg', scale_factor)
