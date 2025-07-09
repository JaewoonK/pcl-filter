#!/usr/bin/env python3
"""
Benchmark script for comparing point cloud filtering methods in Python.

This script benchmarks Statistical Outlier Removal and DBSCAN filtering
on point clouds using Open3D in Python. It measures and reports the
processing time for each method.
"""

import numpy as np
import open3d as o3d
import time
import os
import sys

def create_test_point_cloud(num_normal=1000, num_outliers=50):
    """
    Creates a test point cloud with normal points and outliers.
    
    Args:
        num_normal: Number of normal points to generate
        num_outliers: Number of outlier points to generate
        
    Returns:
        o3d.geometry.PointCloud: The generated point cloud with outliers
    """
    # Create normal points in 3D space, centered around origin
    points = np.random.rand(num_normal, 3) - 0.5  # Range: [-0.5, 0.5]
    
    # Create outlier points that are farther from the center
    outliers = (np.random.rand(num_outliers, 3) - 0.5) * 3  # Range: [-1.5, 1.5]
    
    # Combine normal points and outliers into a single array
    all_points = np.vstack([points, outliers])
    
    # Create Open3D point cloud object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(all_points)
    
    return pcd

def load_point_cloud(filename):
    """
    Load a point cloud from a PCD file.
    
    Args:
        filename: Path to the PCD file
        
    Returns:
        o3d.geometry.PointCloud: The loaded point cloud
    """
    if not os.path.exists(filename):
        print(f"Error: File {filename} not found.")
        return None
        
    return o3d.io.read_point_cloud(filename)

def statistical_outlier_removal(pcd, nb_neighbors=50, std_ratio=1.0):
    """
    Apply Statistical Outlier Removal filter.
    
    Args:
        pcd: Input point cloud
        nb_neighbors: Number of neighbors to analyze (equivalent to meanK in PCL)
        std_ratio: Standard deviation ratio (equivalent to stddevMulThresh in PCL)
        
    Returns:
        tuple: (filtered point cloud, processing time in seconds)
    """
    start_time = time.time()
    
    # Apply Statistical Outlier Removal
    cl, ind = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, 
                                            std_ratio=std_ratio)
    
    end_time = time.time()
    processing_time = end_time - start_time
    
    return cl, processing_time

def dbscan_clustering(pcd, eps=0.15, min_points=10):
    """
    Apply DBSCAN clustering for outlier removal.
    
    Args:
        pcd: Input point cloud
        eps: Maximum distance between points to be considered in the same cluster
        min_points: Minimum number of points required to form a cluster
        
    Returns:
        tuple: (filtered point cloud, processing time in seconds)
    """
    start_time = time.time()
    
    # Compute DBSCAN clustering
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points))
    
    # Filter out noise points (label -1)
    filtered_pcd = o3d.geometry.PointCloud()
    inlier_indices = np.where(labels >= 0)[0]
    filtered_pcd.points = o3d.utility.Vector3dVector(
        np.asarray(pcd.points)[inlier_indices])
    
    end_time = time.time()
    processing_time = end_time - start_time
    
    return filtered_pcd, processing_time

def benchmark(input_file=None, num_runs=5):
    """
    Benchmark both filtering methods.
    
    Args:
        input_file: Path to input PCD file (if None, generates a test cloud)
        num_runs: Number of benchmark runs to average
    """
    # Load or create point cloud
    if input_file and os.path.exists(input_file):
        pcd = load_point_cloud(input_file)
        print(f"Loaded point cloud from {input_file} with {len(pcd.points)} points")
    else:
        pcd = create_test_point_cloud()
        print(f"Generated test point cloud with {len(pcd.points)} points")
        
    # Save for C++ comparison if needed
    if not os.path.exists("input_cloud.pcd"):
        o3d.io.write_point_cloud("input_cloud.pcd", pcd)
        print("Saved point cloud to 'input_cloud.pcd'")
    
    # Parameters (matching the C++ implementation)
    nb_neighbors = 50  # meanK in PCL
    std_ratio = 1.0    # stddevMulThresh in PCL
    eps = 0.15         # cluster_tolerance in PCL
    min_points = 10    # min_cluster_size in PCL
    
    # Benchmark Statistical Outlier Removal
    sor_times = []
    for i in range(num_runs):
        _, time_taken = statistical_outlier_removal(pcd, nb_neighbors, std_ratio)
        sor_times.append(time_taken)
    
    avg_sor_time = sum(sor_times) / len(sor_times)
    print(f"\nStatistical Outlier Removal (Python):")
    print(f"  Parameters: nb_neighbors={nb_neighbors}, std_ratio={std_ratio}")
    print(f"  Average processing time: {avg_sor_time:.6f} seconds")
    
    # Benchmark DBSCAN Clustering
    dbscan_times = []
    for i in range(num_runs):
        _, time_taken = dbscan_clustering(pcd, eps, min_points)
        dbscan_times.append(time_taken)
    
    avg_dbscan_time = sum(dbscan_times) / len(dbscan_times)
    print(f"\nDBSCAN Clustering (Python):")
    print(f"  Parameters: eps={eps}, min_points={min_points}")
    print(f"  Average processing time: {avg_dbscan_time:.6f} seconds")

if __name__ == "__main__":
    input_file = sys.argv[1] if len(sys.argv) > 1 else None
    benchmark(input_file)
