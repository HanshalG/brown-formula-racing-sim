import numpy as np
import matplotlib.pyplot as plt
from svgpathtools import svg2paths
import csv
from matplotlib.collections import LineCollection
from matplotlib.colors import LinearSegmentedColormap
import pandas as pd

def visualize_track_comparison(svg_file, csv_file, scale_factor=1.0, n_points=1000, output_file=None):
    """
    Create a visualization comparing original SVG track with analyzed segments.
    
    Parameters:
    -----------
    svg_file : str
        Path to the input SVG file
    csv_file : str
        Path to the CSV file containing analyzed segments
    scale_factor : float
        Scale factor applied to SVG coordinates
    n_points : int
        Number of points to sample along the SVG path
    output_file : str, optional
        Path to save the visualization. If None, display plot instead.
    """
    
    # Read the original SVG path
    paths, _ = svg2paths(svg_file)
    
    # Sample points along the original SVG path
    original_points = []
    for path in paths:
        for t in np.linspace(0, 1, n_points // len(paths)):
            point = path.point(t)
            original_points.append([point.real * scale_factor, point.imag * scale_factor])
    
    original_points = np.array(original_points)
    # Adjust origin to (0,0)
    original_points -= original_points[0]
    
    # Read the analyzed segments from CSV with proper type conversion
    df = pd.read_csv(csv_file)
    df['Arc Length'] = pd.to_numeric(df['Arc Length'])
    df['Radius of Curvature'] = pd.to_numeric(df['Radius of Curvature'])
    
    # Create figure and axis
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))
    fig.suptitle('Track Analysis Comparison', fontsize=16)
    
    # Plot 1: Original SVG with analyzed segments overlay
    ax1.plot(original_points[:, 0], original_points[:, 1], 
            'k--', alpha=0.5, linewidth=1, label='Original SVG')
    
    # Reconstruct track from segments
    x, y = [0], [0]  # Start at the origin
    heading = np.arctan2(original_points[1, 1] - original_points[0, 1],
                         original_points[1, 0] - original_points[0, 0])
    
    segments = []
    colors = []
    max_radius = 1000  # Cap for visualization
    
    for i, row in df.iterrows():
        length = float(row['Arc Length'])
        radius = min(float(row['Radius of Curvature']), max_radius)
        direction = row['Direction']
        
        if direction == 'Straight':
            # Straight segment
            new_x = x[-1] + length * np.cos(heading)
            new_y = y[-1] + length * np.sin(heading)
        else:
            # Curved segment
            angle = length / radius
            if direction == 'Left':
                center_x = x[-1] - radius * np.sin(heading)
                center_y = y[-1] + radius * np.cos(heading)
                heading += angle
                new_x = center_x + radius * np.sin(heading)
                new_y = center_y - radius * np.cos(heading)
            else:  # right turn
                center_x = x[-1] + radius * np.sin(heading)
                center_y = y[-1] - radius * np.cos(heading)
                heading -= angle
                new_x = center_x - radius * np.sin(heading)
                new_y = center_y + radius * np.cos(heading)
        
        segments.append([(x[-1], y[-1]), (new_x, new_y)])
        colors.append(np.log10(min(radius, max_radius)) / np.log10(max_radius))
        
        x.append(new_x)
        y.append(new_y)
    
    # Create custom colormap (red to blue)
    cmap = LinearSegmentedColormap.from_list('radius_colors', 
                                           ['red', 'yellow', 'blue'])
    
    # Plot segments with colors
    lc = LineCollection(segments, cmap=cmap, array=np.array(colors), 
                       linewidth=2, alpha=0.7)
    ax1.add_collection(lc)
    plt.colorbar(lc, ax=ax1, label='Log10(Radius)')
    
    ax1.set_title('Track Overlay Comparison')
    ax1.set_xlabel('X coordinate')
    ax1.set_ylabel('Y coordinate')
    ax1.axis('equal')
    ax1.grid(True)
    ax1.legend()
    
    # Plot 2: Radius profile
    distances = np.cumsum(df['Arc Length'].values)
    radii = np.minimum(df['Radius of Curvature'].values, max_radius)
    
    direction_colors = {'Left': 'green', 'Right': 'red', 'Straight': 'blue'}
    colors = [direction_colors[d] for d in df['Direction']]
    
    # Plot radius profile with direction colors
    for i in range(len(distances)-1):
        ax2.plot(distances[i:i+2], radii[i:i+2], 
                color=colors[i], linewidth=2, alpha=0.7)
    
    ax2.set_title('Radius Profile')
    ax2.set_xlabel('Distance along track (units)')
    ax2.set_ylabel('Radius of curvature (units)')
    ax2.set_yscale('log')
    ax2.grid(True)

    
    
    # Add direction legend
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], color='red', label='Right turn'),
        Line2D([0], [0], color='green', label='Left turn'),
        Line2D([0], [0], color='blue', label='Straight')
    ]
    ax2.legend(handles=legend_elements)
    
    plt.tight_layout()
    
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
    else:
        plt.show()
    
    return fig

# Example usage:
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='Visualize track analysis comparison')
    parser.add_argument('svg_file', help='Input SVG file path')
    parser.add_argument('csv_file', help='Input CSV file path')
    parser.add_argument('--scale', type=float, default=1.0, 
                      help='Scale factor for SVG coordinates')
    parser.add_argument('--output', type=str, default=None,
                      help='Output file path for visualization')
    
    args = parser.parse_args()
    
    visualize_track_comparison(
        args.svg_file,
        args.csv_file,
        args.scale,
        output_file=args.output
    )
