import cv2
import numpy as np
import matplotlib.pyplot as plt
from scipy import ndimage
import math

def analyze_string_direction_fixed(image_path):
    """
    Analyze string direction from edge - fixed version without ximgproc dependency.
    """
    # Read image
    img = cv2.imread(image_path)
    if img is None:
        raise ValueError(f"Could not read image: {image_path}")
    
    height, width = img.shape[:2]
    
    # Convert to different color spaces
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Get LAB mask (using A channel for orange detection)
    L, A, B = cv2.split(img_lab)
    
    # Threshold A channel (orange has high A values)
    _, lab_mask = cv2.threshold(A, 140, 255, cv2.THRESH_BINARY)
    
    # Clean up the mask
    kernel = np.ones((3, 3), np.uint8)
    lab_mask = cv2.morphologyEx(lab_mask, cv2.MORPH_CLOSE, kernel)
    lab_mask = cv2.morphologyEx(lab_mask, cv2.MORPH_OPEN, kernel)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(lab_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        print("No contours found!")
        return None
    
    # Find the largest contour (assuming it's the string)
    largest_contour = max(contours, key=cv2.contourArea)
    
    # Get contour points for analysis
    contour_points = largest_contour.reshape(-1, 2)
    
    # METHOD 1: Analyze proximity to image edges
    def find_edge_proximity(points, margin=50):
        """Determine which edge the string is closest to"""
        left_dist = np.min(points[:, 0])
        right_dist = width - np.max(points[:, 0])
        top_dist = np.min(points[:, 1])
        bottom_dist = height - np.max(points[:, 1])
        
        distances = {
            'left': left_dist,
            'right': right_dist,
            'top': top_dist,
            'bottom': bottom_dist
        }
        
        closest_edge = min(distances, key=distances.get)
        return closest_edge, distances[closest_edge], distances
    
    # METHOD 2: Find string orientation using PCA
    def find_string_orientation(points):
        """Use PCA to find the main direction of the string"""
        # Center the points
        mean = np.mean(points, axis=0)
        centered = points - mean
        
        # Compute covariance matrix
        if len(centered) < 2:
            return 0, np.array([1, 0]), mean
        
        cov = np.cov(centered.T)
        
        # Get eigenvectors (principal components)
        try:
            eigenvalues, eigenvectors = np.linalg.eig(cov)
            # The eigenvector with largest eigenvalue is the principal direction
            principal_idx = np.argmax(eigenvalues)
            principal_vector = eigenvectors[:, principal_idx]
            
            # Calculate angle in degrees
            angle = math.degrees(math.atan2(principal_vector[1], principal_vector[0]))
            
            # Normalize angle to 0-360
            angle = angle % 360
        except:
            angle = 0
            principal_vector = np.array([1, 0])
        
        return angle, principal_vector, mean
    
    # METHOD 3: Determine entry point from edge
    def find_entry_point(contour, edge, margin=50):
        """Find where the string enters from the edge"""
        edge_points = []
        contour = contour.reshape(-1, 2)
        
        for point in contour:
            x, y = point
            
            if edge == 'left' and x < margin:
                edge_points.append([x, y])
            elif edge == 'right' and x > (width - margin):
                edge_points.append([x, y])
            elif edge == 'top' and y < margin:
                edge_points.append([x, y])
            elif edge == 'bottom' and y > (height - margin):
                edge_points.append([x, y])
        
        if edge_points:
            # Return the average of edge points
            edge_points = np.array(edge_points)
            entry_point = np.mean(edge_points, axis=0)
            return entry_point
        else:
            # Find closest point to edge
            min_dist = float('inf')
            entry_point = None
            
            for point in contour:
                x, y = point
                if edge == 'left':
                    dist = x
                elif edge == 'right':
                    dist = width - x
                elif edge == 'top':
                    dist = y
                else:  # bottom
                    dist = height - y
                
                if dist < min_dist:
                    min_dist = dist
                    entry_point = np.array([x, y])
            
            return entry_point if entry_point is not None else np.array([0, 0])
    
    # METHOD 4: Determine direction based on PCA and edge
    def find_direction_from_edge(pca_angle, pca_vector, entry_point, edge, img_center):
        """Determine which way the string goes from the edge"""
        # For left edge: string should go rightward (positive x)
        # For right edge: string should go leftward (negative x)
        # For top edge: string should go downward (positive y)
        # For bottom edge: string should go upward (negative y)
        
        # Flip PCA vector if it points the wrong way
        corrected_vector = pca_vector.copy()
        
        if edge == 'left':
            # String should generally go right (positive x)
            if pca_vector[0] < 0:
                corrected_vector = -pca_vector
        elif edge == 'right':
            # String should generally go left (negative x)
            if pca_vector[0] > 0:
                corrected_vector = -pca_vector
        elif edge == 'top':
            # String should generally go down (positive y)
            if pca_vector[1] < 0:
                corrected_vector = -pca_vector
        elif edge == 'bottom':
            # String should generally go up (negative y)
            if pca_vector[1] > 0:
                corrected_vector = -pca_vector
        
        # Calculate angle from corrected vector
        angle = math.degrees(math.atan2(corrected_vector[1], corrected_vector[0]))
        angle = angle % 360
        
        return corrected_vector, angle
    
    # METHOD 5: Find direction using contour points near entry
    def find_local_direction(contour_points, entry_point, sample_radius=100):
        """Find direction by analyzing points near the entry point"""
        # Find points within sample_radius of entry point
        distances = np.linalg.norm(contour_points - entry_point, axis=1)
        nearby_indices = np.where(distances < sample_radius)[0]
        
        if len(nearby_indices) < 10:
            # Not enough points, use PCA on all points
            angle, vector, _ = find_string_orientation(contour_points)
            return vector, angle
        
        nearby_points = contour_points[nearby_indices]
        
        # Sort by distance
        sorted_indices = np.argsort(distances[nearby_indices])
        nearby_points = nearby_points[sorted_indices]
        
        # Take the farthest point within radius as direction indicator
        if len(nearby_points) > 1:
            far_point = nearby_points[-1]
            direction_vector = far_point - entry_point
            
            # Normalize
            norm = np.linalg.norm(direction_vector)
            if norm > 0:
                direction_vector = direction_vector / norm
            
            angle = math.degrees(math.atan2(direction_vector[1], direction_vector[0]))
            angle = angle % 360
            
            return direction_vector, angle
        
        return np.array([1, 0]), 0
    
    # Perform analysis
    closest_edge, edge_distance, all_distances = find_edge_proximity(contour_points)
    
    # Find entry point
    entry_point = find_entry_point(largest_contour, closest_edge)
    
    # Get PCA orientation
    pca_angle, pca_vector, center = find_string_orientation(contour_points)
    
    # Get image center for reference
    img_center = np.array([width/2, height/2])
    
    # Determine direction using PCA corrected for edge
    corrected_vector, corrected_angle = find_direction_from_edge(
        pca_angle, pca_vector, entry_point, closest_edge, img_center
    )
    
    # Also get local direction
    local_vector, local_angle = find_local_direction(contour_points, entry_point)
    
    # Choose which direction to use (prefer local if we have enough points)
    if len(contour_points) > 50:
        final_vector = local_vector
        final_angle = local_angle
    else:
        final_vector = corrected_vector
        final_angle = corrected_angle
    
    # METHOD 6: Visualize with multiple arrows
    def visualize_direction_comprehensive(image, entry_point, vectors, angles, edge_name):
        """Create visualization with multiple direction indicators"""
        vis = image.copy()
        
        # Draw contour
        cv2.drawContours(vis, [largest_contour], -1, (0, 255, 0), 2)
        
        # Draw entry point
        entry_int = (int(entry_point[0]), int(entry_point[1]))
        cv2.circle(vis, entry_int, 10, (255, 0, 0), -1)
        
        # Draw PCA direction (green)
        pca_end = (int(entry_point[0] + pca_vector[0] * 150),
                  int(entry_point[1] + pca_vector[1] * 150))
        cv2.arrowedLine(vis, entry_int, pca_end, (0, 255, 0), 3, tipLength=0.3)
        
        # Draw corrected direction (red)
        corr_end = (int(entry_point[0] + corrected_vector[0] * 150),
                   int(entry_point[1] + corrected_vector[1] * 150))
        cv2.arrowedLine(vis, entry_int, corr_end, (0, 0, 255), 3, tipLength=0.3)
        
        # Draw local direction (yellow)
        local_end = (int(entry_point[0] + local_vector[0] * 150),
                    int(entry_point[1] + local_vector[1] * 150))
        cv2.arrowedLine(vis, entry_int, local_end, (0, 255, 255), 3, tipLength=0.3)
        
        # Add text
        text_y = 40
        cv2.putText(vis, f"Edge: {edge_name}", (20, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        text_y += 40
        cv2.putText(vis, f"PCA Angle: {pca_angle:.1f}°", (20, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        text_y += 40
        cv2.putText(vis, f"Corrected: {corrected_angle:.1f}°", (20, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        text_y += 40
        cv2.putText(vis, f"Local: {local_angle:.1f}°", (20, text_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        
        # Legend
        legend_y = height - 150
        cv2.putText(vis, "PCA (Green)", (20, legend_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        legend_y += 30
        cv2.putText(vis, "Corrected (Red)", (20, legend_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        legend_y += 30
        cv2.putText(vis, "Local (Yellow)", (20, legend_y), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        return vis
    
    # Create visualization
    vectors = {
        'pca': pca_vector,
        'corrected': corrected_vector,
        'local': local_vector
    }
    
    angles = {
        'pca': pca_angle,
        'corrected': corrected_angle,
        'local': local_angle
    }
    
    visualization = visualize_direction_comprehensive(
        img_rgb, entry_point, vectors, angles, closest_edge
    )
    
    # Create analysis plot
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    
    # Original image
    axes[0, 0].imshow(img_rgb)
    axes[0, 0].set_title('Original Image')
    axes[0, 0].axis('off')
    
    # LAB mask
    axes[0, 1].imshow(lab_mask, cmap='gray')
    axes[0, 1].set_title('LAB Mask (Orange Detection)')
    axes[0, 1].axis('off')
    
    # Grayscale
    axes[0, 2].imshow(img_gray, cmap='gray')
    axes[0, 2].set_title('Grayscale')
    axes[0, 2].axis('off')
    
    # Visualization
    axes[1, 0].imshow(visualization)
    axes[1, 0].set_title('Direction Analysis')
    axes[1, 0].axis('off')
    
    # Edge distances bar chart
    edges = list(all_distances.keys())
    distances = [all_distances[e] for e in edges]
    
    axes[1, 1].bar(edges, distances, color=['red' if e == closest_edge else 'blue' for e in edges])
    axes[1, 1].set_title('Distance to Edges')
    axes[1, 1].set_ylabel('Pixels')
    axes[1, 1].set_xlabel('Edge')
    
    # Direction angles polar plot
    axes[1, 2] = plt.subplot(236, polar=True)
    
    angles_to_plot = [pca_angle, corrected_angle, local_angle]
    colors = ['green', 'red', 'yellow']
    labels = ['PCA', 'Corrected', 'Local']
    
    for angle, color, label in zip(angles_to_plot, colors, labels):
        theta = np.radians(angle)
        axes[1, 2].arrow(theta, 0, 0, 0.8, alpha=0.7, width=0.03,
                        edgecolor=color, facecolor=color, lw=2, 
                        label=label)
    
    axes[1, 2].set_theta_zero_location("E")
    axes[1, 2].set_theta_direction(-1)
    axes[1, 2].set_title('Direction Angles', pad=20)
    axes[1, 2].set_ylim(0, 1)
    axes[1, 2].grid(True)
    axes[1, 2].legend(loc='upper right')
    
    plt.tight_layout()
    plt.show()
    
    # Print comprehensive results
    print("=" * 60)
    print("STRING DIRECTION ANALYSIS - COMPREHENSIVE")
    print("=" * 60)
    print(f"\nImage Dimensions: {width} x {height}")
    print(f"Contour points: {len(contour_points)}")
    print(f"String area: {cv2.contourArea(largest_contour):.0f} pixels")
    
    print(f"\nEdge Proximity Analysis:")
    for edge, dist in all_distances.items():
        marker = " ← CLOSEST" if edge == closest_edge else ""
        print(f"  {edge.upper()} edge: {dist:.1f} pixels{marker}")
    
    print(f"\nEntry Point Analysis:")
    print(f"  Entry edge: {closest_edge.upper()}")
    print(f"  Entry point coordinates: ({entry_point[0]:.1f}, {entry_point[1]:.1f})")
    
    print(f"\nDirection Analysis:")
    print(f"  1. PCA Analysis: {pca_angle:.1f}°")
    print(f"     Vector: [{pca_vector[0]:.3f}, {pca_vector[1]:.3f}]")
    print(f"  2. Edge-corrected: {corrected_angle:.1f}°")
    print(f"     Vector: [{corrected_vector[0]:.3f}, {corrected_vector[1]:.3f}]")
    print(f"  3. Local analysis: {local_angle:.1f}°")
    print(f"     Vector: [{local_vector[0]:.3f}, {local_vector[1]:.3f}]")
    
    # Determine the most reliable angle
    print(f"\nRECOMMENDED DIRECTION:")
    
    # Check angle consistency
    angle_diff1 = abs(pca_angle - corrected_angle) % 180
    angle_diff2 = abs(corrected_angle - local_angle) % 180
    
    if angle_diff1 < 30 and angle_diff2 < 30:
        # All angles are consistent
        avg_angle = np.mean([pca_angle, corrected_angle, local_angle])
        print(f"  All methods agree. Average angle: {avg_angle:.1f}°")
        recommended_angle = avg_angle
    elif len(contour_points) > 100:
        # Large contour, prefer local analysis
        print(f"  Large string detected. Using local analysis: {local_angle:.1f}°")
        recommended_angle = local_angle
    else:
        # Prefer PCA
        print(f"  Using PCA analysis: {pca_angle:.1f}°")
        recommended_angle = pca_angle
    
    # Convert to cardinal direction
    def angle_to_cardinal(angle):
        """Convert angle to cardinal direction"""
        # Normalize to 0-360
        angle = angle % 360
        
        directions = [
            (0, 22.5, "East"),
            (22.5, 67.5, "Northeast"),
            (67.5, 112.5, "North"),
            (112.5, 157.5, "Northwest"),
            (157.5, 202.5, "West"),
            (202.5, 247.5, "Southwest"),
            (247.5, 292.5, "South"),
            (292.5, 337.5, "Southeast"),
            (337.5, 360, "East")
        ]
        
        for start, end, name in directions:
            if start <= angle < end:
                return name
        
        return "Unknown"
    
    cardinal = angle_to_cardinal(recommended_angle)
    
    print(f"\nINTERPRETATION:")
    print(f"  The orange string enters from the {closest_edge} edge.")
    print(f"  It extends {cardinal}ward from the entry point.")
    print(f"  Recommended tracking angle: {recommended_angle:.1f}°")
    
    # Special case: If PCA says 3.2° but previous analysis said 90°
    if abs(pca_angle - 3.2) < 5 and abs(recommended_angle - 90) < 5:
        print(f"\nNOTE: PCA suggests the string is nearly horizontal (~3°),")
        print(f"      while local analysis suggests vertical (~90°).")
        print(f"      This could mean the string has both horizontal and vertical segments.")
    
    return {
        'original': img_rgb,
        'lab_mask': lab_mask,
        'contour': largest_contour,
        'closest_edge': closest_edge,
        'entry_point': entry_point,
        'pca_angle': pca_angle,
        'pca_vector': pca_vector,
        'corrected_angle': corrected_angle,
        'corrected_vector': corrected_vector,
        'local_angle': local_angle,
        'local_vector': local_vector,
        'recommended_angle': recommended_angle,
        'cardinal_direction': cardinal,
        'visualization': visualization
    }

def simple_direction_finder(image_path):
    """
    Simple function to get just the string direction.
    """
    results = analyze_string_direction_fixed(image_path)
    
    if results:
        print("\n" + "=" * 50)
        print("SIMPLE DIRECTION OUTPUT:")
        print("=" * 50)
        print(f"String enters from: {results['closest_edge'].upper()} edge")
        print(f"Entry point: ({results['entry_point'][0]:.0f}, {results['entry_point'][1]:.0f})")
        print(f"Direction: {results['recommended_angle']:.1f}° ({results['cardinal_direction']})")
        print(f"Direction vector: [{results['corrected_vector'][0]:.3f}, {results['corrected_vector'][1]:.3f}]")
        
        # Show simple visualization
        cv2.imshow('Detected String', cv2.cvtColor(results['original'], cv2.COLOR_RGB2BGR))
        cv2.imshow('Direction Overlay', cv2.cvtColor(results['visualization'], cv2.COLOR_RGB2BGR))
        
        # Draw simple arrow on original
        simple_vis = results['original'].copy()
        entry = (int(results['entry_point'][0]), int(results['entry_point'][1]))
        arrow_len = 200
        arrow_end = (
            int(entry[0] + results['corrected_vector'][0] * arrow_len),
            int(entry[1] + results['corrected_vector'][1] * arrow_len)
        )
        cv2.arrowedLine(simple_vis, entry, arrow_end, (0, 0, 255), 4, tipLength=0.3)
        cv2.putText(simple_vis, f"{results['recommended_angle']:.1f}°", 
                   (entry[0] + 30, entry[1] - 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        cv2.imshow('Simple Direction', cv2.cvtColor(simple_vis, cv2.COLOR_RGB2BGR))
        
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        
        return results['recommended_angle'], results['corrected_vector']
    
    return None, None

# Example usage
if __name__ == "__main__":
    # image_path = "string_east.jpeg"
    # image_path = "string_west.jpeg"
    # image_path = "string_north.jpeg"
    # image_path = "string_south.jpeg"
    image_path = "weird.jpg"
    
    print("Analyzing string direction with improved algorithm...")
    print("-" * 60)
    
    # Full analysis with visualization
    results = analyze_string_direction_fixed(image_path)
    
    # Simple output
    angle, vector = simple_direction_finder(image_path)
    
    if angle is not None:
        print(f"\nFinal result: The string goes at {angle:.1f}° from the entry point.")
        
        # Save direction information
        with open('string_direction.txt', 'w') as f:
            f.write(f"String Direction Analysis\n")
            f.write(f"=======================\n")
            f.write(f"Image: {image_path}\n")
            f.write(f"Entry edge: {results['closest_edge']}\n")
            f.write(f"Entry point: {results['entry_point'][0]:.1f}, {results['entry_point'][1]:.1f}\n")
            f.write(f"Recommended angle: {angle:.1f}°\n")
            f.write(f"Direction vector: [{vector[0]:.4f}, {vector[1]:.4f}]\n")
            f.write(f"Cardinal direction: {results['cardinal_direction']}\n")
        
        print("Direction saved to 'string_direction.txt'")