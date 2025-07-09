/**
 * @file main.cpp
 * @brief Point Cloud Filtering using Statistical Outlier Removal
 * 
 * This program demonstrates how to use PCL's StatisticalOutlierRemoval filter
 * to clean point cloud data by removing outliers. It loads a point cloud from
 * a PCD file, applies the filter, and saves the filtered result.
 *
 * Statistical Outlier Removal is a technique for identifying and removing outlier
 * points that are statistically distant from the rest of the data. It is particularly
 * useful for cleaning noisy sensor data.
 */

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

/**
 * @brief Main function for point cloud filtering
 * 
 * Loads a point cloud from a PCD file, applies statistical outlier removal,
 * and saves the filtered result.
 * 
 * @return int Exit code
 */
int main() {
    // Create a PointCloud pointer
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // Load the PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("input_cloud.pcd", *cloud) == -1) {
        PCL_ERROR("Couldn't read file input_cloud.pcd \n");
        return (-1);
    }

    std::cout << "Loaded "
              << cloud->width * cloud->height
              << " data points from input_cloud.pcd" << std::endl;

    /**
     * Statistical Outlier Removal Filter
     * 
     * The algorithm works as follows:
     * 1. For each point, it finds the k nearest neighbors (k = meanK)
     * 2. Calculates the average distance to those neighbors
     * 3. Computes the mean and standard deviation of all these average distances
     * 4. Points whose average distance is outside μ ± (stddev_mult * σ) are removed
     * 
     * Parameters explanation:
     * - meanK: The number of nearest neighbors to consider for each point.
     *   Larger values make the algorithm more robust but also more computationally expensive.
     *   Typical values range from 10-100 depending on the point cloud density.
     * 
     * - stddev_mult: The standard deviation multiplier threshold.
     *   Points with mean distances outside μ ± (stddev_mult * σ) are considered outliers.
     *   Smaller values result in more aggressive filtering (more points removed).
     *   Typical values range from 0.5-2.0.
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
    // Apply the filter
    sor.filter(*cloud_filtered);

    std::cout << "Cloud after filtering: " << std::endl;
    std::cout << *cloud_filtered << std::endl;

    // Save the filtered cloud
    pcl::io::savePCDFile<pcl::PointXYZ>("filtered_cloud.pcd", *cloud_filtered);

    return (0);
}
