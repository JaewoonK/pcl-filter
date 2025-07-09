/**
 * @file visualize_clouds.cpp
 * @brief Visualizes point clouds processed with different outlier removal methods
 * 
 * This program demonstrates and compares two different methods for processing point clouds:
 * 1. Statistical Outlier Removal (SOR) - Removes outliers based on statistical analysis
 * 2. DBSCAN Clustering - Density-Based Spatial Clustering of Applications with Noise
 *
 * Statistical Outlier Removal works by:
 * 1. Computing the mean distance from each point to its k nearest neighbors
 * 2. Calculating the mean and standard deviation of all these distances
 * 3. Classifying points as outliers if their mean distance is outside a defined threshold
 *    (mean + stddev_mult * stddev)
 *
 * DBSCAN Clustering works by:
 * 1. Grouping points that are closely packed together (points with many neighbors)
 * 2. Marking points in low-density regions as outliers (noise)
 * 3. Identifying clusters based on density connectivity
 * 
 * @author PCL Filtering Project
 * @date July 2025
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/extract_indices.h>
#include <thread>
#include <chrono>
#include <vector>

/**
 * @brief Apply DBSCAN clustering to identify and remove outliers
 * 
 * DBSCAN (Density-Based Spatial Clustering of Applications with Noise) is a clustering
 * algorithm that groups points that are closely packed together, while marking points
 * in low-density regions as outliers (noise).
 * 
 * @param cloud Input point cloud
 * @param cluster_tolerance Distance threshold for clustering (epsilon parameter in DBSCAN)
 * @param min_cluster_size Minimum number of points required to form a cluster
 * @param max_cluster_size Maximum number of points allowed in a cluster
 * @return Filtered point cloud with outliers removed
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr applyDBSCAN(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                              float cluster_tolerance = 0.05,
                                              int min_cluster_size = 10,
                                              int max_cluster_size = 10000) {
    // Create the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    // Create cluster indices vector
    std::vector<pcl::PointIndices> cluster_indices;
    
    // Create the DBSCAN clustering object
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance); // Set the spatial cluster tolerance (epsilon)
    ec.setMinClusterSize(min_cluster_size);    // Set the minimum cluster size
    ec.setMaxClusterSize(max_cluster_size);    // Set the maximum cluster size
    ec.setSearchMethod(tree);                  // Set the search method
    ec.setInputCloud(cloud);                   // Set the input cloud
    ec.extract(cluster_indices);               // Extract the clusters

    // Create a new cloud to hold all points in clusters (non-outliers)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Create point indices for all clustered points
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    
    // Combine all cluster indices into a single set of inliers
    for (const auto& cluster : cluster_indices) {
        inliers->indices.insert(inliers->indices.end(), 
                               cluster.indices.begin(), 
                               cluster.indices.end());
    }
    
    // Extract the inliers
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false); // Extract the inliers
    extract.filter(*cloud_filtered);
    
    return cloud_filtered;
}

/**
 * @brief Main function for point cloud visualization and comparison of filtering methods
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments (optional: path to input PCD file)
 * @return int Exit code
 */
int main(int argc, char** argv) {
    // Create point cloud pointers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_sor(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_dbscan(new pcl::PointCloud<pcl::PointXYZ>);

    // Check if input file is provided
    std::string input_file = "input_cloud.pcd";
    if (argc > 1) {
        input_file = argv[1];
    }

    // Load the PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", input_file.c_str());
        return (-1);
    }

    std::cout << "Loaded " << cloud->width * cloud->height
              << " data points from " << input_file << std::endl;

    /**
     * Statistical Outlier Removal Filter
     * 
     * This filter identifies and removes outliers from point clouds based on statistical analysis
     * of each point's neighborhood. The algorithm works as follows:
     * 
     * 1. For each point p in the cloud:
     *    a. Find its k nearest neighbors (k = meanK parameter)
     *    b. Calculate the mean distance from p to all its neighbors
     * 
     * 2. Compute the mean (μ) and standard deviation (σ) of all these mean distances
     * 
     * 3. Define an interval [μ - α*σ, μ + α*σ] where α is the stddev multiplier
     * 
     * 4. Points whose mean distance falls outside this interval are classified as outliers
     *    and removed from the point cloud
     * 
     * Parameters:
     * - meanK: Number of nearest neighbors to use (higher values = more aggressive smoothing)
     * - stddev_mult: Standard deviation multiplier (lower values = more points removed)
     */
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    // Set the number of nearest neighbors to analyze for each point
    // Higher values consider more points in the local neighborhood
    sor.setMeanK(50);
    // Set the standard deviation multiplier threshold
    // Points with mean distances outside μ ± (stddev_mult * σ) are removed
    // Lower values remove more points
    sor.setStddevMulThresh(1.0);
    // Apply the Statistical Outlier Removal filter
    sor.filter(*cloud_filtered_sor);

    std::cout << "Cloud after Statistical Outlier Removal filtering: " << std::endl;
    std::cout << "Original size: " << cloud->size() << " points" << std::endl;
    std::cout << "Filtered size: " << cloud_filtered_sor->size() << " points" << std::endl;
    std::cout << "Points removed: " << (cloud->size() - cloud_filtered_sor->size()) << std::endl;

    // Save the SOR filtered cloud
    pcl::io::savePCDFile<pcl::PointXYZ>("filtered_cloud_sor.pcd", *cloud_filtered_sor);
    std::cout << "Saved SOR filtered cloud to 'filtered_cloud_sor.pcd'" << std::endl;
    
    /**
     * DBSCAN Clustering for Outlier Removal
     * 
     * DBSCAN is a density-based clustering algorithm that can be used for outlier detection.
     * It works by grouping points that are closely packed together and marking points
     * in low-density regions as outliers (noise).
     * 
     * Parameters:
     * - cluster_tolerance: The maximum distance between two points to be considered in the same cluster
     * - min_cluster_size: The minimum number of points required to form a cluster
     * - max_cluster_size: The maximum number of points allowed in a cluster
     */
    std::cout << "\nApplying DBSCAN clustering..." << std::endl;
    // Apply DBSCAN clustering
    cloud_filtered_dbscan = applyDBSCAN(cloud, 0.15, 10, 10000);
    
    std::cout << "Cloud after DBSCAN clustering: " << std::endl;
    std::cout << "Original size: " << cloud->size() << " points" << std::endl;
    std::cout << "Filtered size: " << cloud_filtered_dbscan->size() << " points" << std::endl;
    std::cout << "Points removed: " << (cloud->size() - cloud_filtered_dbscan->size()) << std::endl;

    // Save the DBSCAN filtered cloud
    pcl::io::savePCDFile<pcl::PointXYZ>("filtered_cloud_dbscan.pcd", *cloud_filtered_dbscan);
    std::cout << "Saved DBSCAN filtered cloud to 'filtered_cloud_dbscan.pcd'" << std::endl;

    /**
     * Point Cloud Visualization
     * 
     * Creates a three-viewport visualization to compare:
     * 1. Original point cloud
     * 2. Statistical Outlier Removal filtered cloud
     * 3. DBSCAN filtered cloud
     * 
     * This allows for visual comparison between the two filtering methods.
     * 
     * The visualization uses PCL's PCLVisualizer class which provides interactive 3D visualization
     * capabilities including rotation, zooming, and panning.
     */
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Point Cloud Filtering Comparison"));
    viewer->setBackgroundColor(0, 0, 0);
    
    // Define viewports - split the window into three views
    int v1(0);  // Left viewport for original cloud
    int v2(1);  // Middle viewport for SOR filtered cloud
    int v3(2);  // Right viewport for DBSCAN filtered cloud
    viewer->createViewPort(0.0, 0.0, 0.33, 1.0, v1);
    viewer->createViewPort(0.33, 0.0, 0.66, 1.0, v2);
    viewer->createViewPort(0.66, 0.0, 1.0, 1.0, v3);
    
    // Set background colors for each viewport
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v1);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
    viewer->setBackgroundColor(0.1, 0.1, 0.1, v3);
    
    // Add text labels to viewports
    viewer->addText("Original Cloud", 10, 10, 16, 1.0, 1.0, 1.0, "original_label", v1);
    viewer->addText("Statistical Outlier Removal", 10, 10, 16, 1.0, 1.0, 1.0, "sor_label", v2);
    viewer->addText("DBSCAN Clustering", 10, 10, 16, 1.0, 1.0, 1.0, "dbscan_label", v3);
    
    // Color the original cloud white
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white_color(cloud, 255, 255, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, white_color, "original_cloud", v1);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    
    // Color the SOR filtered cloud green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(cloud_filtered_sor, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered_sor, green_color, "sor_cloud", v2);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "sor_cloud");
    
    // Color the DBSCAN filtered cloud blue
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue_color(cloud_filtered_dbscan, 0, 0, 255);
    viewer->addPointCloud<pcl::PointXYZ>(cloud_filtered_dbscan, blue_color, "dbscan_cloud", v3);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "dbscan_cloud");
    
    // Add coordinate systems to all viewports
    viewer->addCoordinateSystem(0.1, "coord1", v1);
    viewer->addCoordinateSystem(0.1, "coord2", v2);
    viewer->addCoordinateSystem(0.1, "coord3", v3);
    
    // Set camera position for both viewports
    viewer->setCameraPosition(0, 0, 3, 0, 0, 0, 0, 1, 0);
    
    std::cout << "\nVisualizing point clouds with different filtering methods." << std::endl;
    std::cout << "Left: Original Cloud | Middle: Statistical Outlier Removal | Right: DBSCAN Clustering" << std::endl;
    std::cout << "Press 'q' to exit..." << std::endl;
    
    // Enter visualization loop
    while (!viewer->wasStopped()) {
        viewer->spinOnce(100);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    return 0;
}
