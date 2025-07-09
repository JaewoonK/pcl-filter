/**
 * @file create_test_cloud.cpp
 * @brief Creates a synthetic point cloud with intentional outliers for testing
 * 
 * This program generates a point cloud with two distinct distributions:
 * 1. A core set of points within a normal distribution range
 * 2. A smaller set of outlier points that are positioned farther from the center
 * 
 * The resulting point cloud is saved as a PCD file that can be used to test
 * outlier removal algorithms like StatisticalOutlierRemoval.
 * 
 * @author PCL Filtering Project
 * @date July 2025
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <random>

/**
 * @brief Main function for generating a test point cloud with outliers
 * 
 * Creates a point cloud with 1000 normal points and 50 outlier points.
 * Normal points are distributed within [-0.5, 0.5] range in all dimensions.
 * Outlier points are distributed within [-1.5, 1.5] range, making them
 * statistically distant from the core points.
 * 
 * @return int Exit code
 */
int main() {
    // Create point cloud object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Set basic parameters
    cloud->width = 1050;   // Total number of points (1000 normal + 50 outliers)
    cloud->height = 1;     // Unorganized point cloud (not a structured grid)
    cloud->is_dense = false; // May contain invalid points (NaN/Inf)
    cloud->points.resize(cloud->width * cloud->height);
    
    /**
     * Random Point Generation
     * 
     * We use two different distributions to create a point cloud with clear outliers:
     * 1. Normal points: Uniformly distributed in [-0.5, 0.5] range
     * 2. Outlier points: Uniformly distributed in [-1.5, 1.5] range
     * 
     * This creates a point cloud where the outliers are statistically distant from
     * the core points, making them good candidates for removal by the
     * StatisticalOutlierRemoval filter.
     */
    // Random number generators
    std::random_device rd;  // Used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()
    
    // Distribution for normal points (core cluster)
    std::uniform_real_distribution<> normal_dist(-0.5, 0.5);
    
    // Distribution for outlier points (wider range = farther from core)
    std::uniform_real_distribution<> outlier_dist(-1.5, 1.5);
    
    // Generate 1000 normal points that form the core of the point cloud
    for (size_t i = 0; i < 1000; ++i) {
        cloud->points[i].x = normal_dist(gen);
        cloud->points[i].y = normal_dist(gen);
        cloud->points[i].z = normal_dist(gen);
    }
    
    // Generate 50 outlier points that are statistically distant from the core
    for (size_t i = 1000; i < 1050; ++i) {
        cloud->points[i].x = outlier_dist(gen);
        cloud->points[i].y = outlier_dist(gen);
        cloud->points[i].z = outlier_dist(gen);
    }
    
    std::cout << "Generated point cloud with " << cloud->size() 
              << " points (1000 normal + 50 outliers)" << std::endl;
    
    // Save the point cloud
    pcl::io::savePCDFileASCII("input_cloud.pcd", *cloud);
    std::cout << "Saved point cloud to 'input_cloud.pcd'" << std::endl;
    
    return 0;
}
