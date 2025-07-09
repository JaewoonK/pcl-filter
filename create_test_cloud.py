"""
Point Cloud Generator for Testing Statistical Outlier Removal

This script creates a synthetic point cloud with intentional outliers for testing
filtering algorithms. It uses Open3D to generate and visualize the point cloud.

The generated point cloud consists of:
1. 1000 normal points distributed within a cube centered at the origin
2. 50 outlier points distributed farther from the center

The resulting point cloud is saved as a PCD file that can be used with PCL.

Author: PCL Filtering Project
Date: July 2025
"""

import numpy as np
import open3d as o3d

def create_test_point_cloud():
    """
    Creates a test point cloud with normal points and outliers.
    
    The function generates two sets of points:
    - Normal points: 1000 points uniformly distributed in [-0.5, 0.5] range
    - Outlier points: 50 points distributed in a wider range (3x the normal range)
    
    Returns:
        o3d.geometry.PointCloud: The generated point cloud with outliers
    """
    # Create 1000 normal points in 3D space, centered around origin
    # These points form the core of the point cloud
    points = np.random.rand(1000, 3) - 0.5  # Range: [-0.5, 0.5]
    
    # Create 50 outlier points that are farther from the center
    # The factor of 3 makes these points statistically distant from the core
    outliers = (np.random.rand(50, 3) - 0.5) * 3  # Range: [-1.5, 1.5]
    
    # Combine normal points and outliers into a single array
    all_points = np.vstack([points, outliers])
    
    # Create Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_points)
    
    return pcd

# Generate the point cloud
point_cloud = create_test_point_cloud()

# Save to PCD file format (compatible with PCL)
o3d.io.write_point_cloud("input_cloud.pcd", point_cloud)
print("Created input_cloud.pcd with 1050 points (including 50 outliers)")

# Visualize the point cloud (optional)
# This will open a window showing the generated point cloud
# You can rotate, zoom, and pan using the mouse
o3d.visualization.draw_geometries([point_cloud], 
                                 window_name="Point Cloud with Outliers",
                                 width=800, 
                                 height=600)
