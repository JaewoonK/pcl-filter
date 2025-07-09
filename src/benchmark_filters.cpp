/**
 * @file benchmark_filters.cpp
 * @brief Benchmark program for comparing point cloud filtering methods in C++
 * 
 * This program benchmarks Statistical Outlier Removal and DBSCAN filtering
 * on point clouds using PCL in C++. It measures and reports the
 * processing time for each method.
 */

#include <iostream>
#include <chrono>
#include <vector>
#include <numeric>
#include <iomanip>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>

/**
 * Apply Statistical Outlier Removal filter and measure processing time
 * 
 * @param cloud Input point cloud
 * @param meanK Number of neighbors to analyze
 * @param stddev_mult Standard deviation multiplier threshold
 * @return Pair of filtered cloud and processing time in seconds
 */
std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, double> 
applyStatisticalOutlierRemoval(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, 
                              int meanK, double stddev_mult) {
    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    sor.setInputCloud(cloud);
    sor.setMeanK(meanK);
    sor.setStddevMulThresh(stddev_mult);
    sor.filter(*cloud_filtered);
    
    // End timing
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    
    return {cloud_filtered, elapsed.count()};
}

/**
 * Apply DBSCAN clustering for outlier removal and measure processing time
 * 
 * @param cloud Input point cloud
 * @param cluster_tolerance Maximum distance between points in a cluster
 * @param min_cluster_size Minimum number of points in a cluster
 * @param max_cluster_size Maximum number of points in a cluster
 * @return Pair of filtered cloud and processing time in seconds
 */
std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, double> 
applyDBSCAN(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
           double cluster_tolerance, int min_cluster_size, int max_cluster_size) {
    // Start timing
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // Create result cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Create KdTree for searching
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);
    
    // Perform DBSCAN clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    
    // Extract all points belonging to clusters (non-outliers)
    for (const auto& cluster : cluster_indices) {
        for (const auto& idx : cluster.indices) {
            cloud_filtered->push_back((*cloud)[idx]);
        }
    }
    
    // End timing
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    
    return {cloud_filtered, elapsed.count()};
}

/**
 * Main function for benchmarking point cloud filtering methods
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments (optional: input PCD file path)
 * @return Exit code
 */
int main(int argc, char** argv) {
    // Load the input point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    // Check if a file path was provided
    std::string input_file = "input_cloud.pcd";
    if (argc > 1) {
        input_file = argv[1];
    }
    
    // Load the PCD file
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", input_file.c_str());
        return -1;
    }
    
    std::cout << "Loaded " << cloud->size() << " points from " << input_file << std::endl;
    
    // Parameters (matching the Python implementation)
    int meanK = 50;                // nb_neighbors in Python
    double stddev_mult = 1.0;      // std_ratio in Python
    double cluster_tolerance = 0.15; // eps in Python
    int min_cluster_size = 10;     // min_points in Python
    int max_cluster_size = 10000;  // not directly used in Python
    
    // Number of benchmark runs
    const int num_runs = 5;
    
    // Benchmark Statistical Outlier Removal
    std::vector<double> sor_times;
    for (int i = 0; i < num_runs; ++i) {
        auto result = applyStatisticalOutlierRemoval(cloud, meanK, stddev_mult);
        double time_taken = result.second;
        sor_times.push_back(time_taken);
    }
    
    double avg_sor_time = std::accumulate(sor_times.begin(), sor_times.end(), 0.0) / sor_times.size();
    std::cout << "\nStatistical Outlier Removal (C++):" << std::endl;
    std::cout << "  Parameters: meanK=" << meanK << ", stddev_mult=" << stddev_mult << std::endl;
    std::cout << "  Average processing time: " << std::fixed << std::setprecision(6) << avg_sor_time << " seconds" << std::endl;
    
    // Benchmark DBSCAN Clustering
    std::vector<double> dbscan_times;
    for (int i = 0; i < num_runs; ++i) {
        auto result = applyDBSCAN(cloud, cluster_tolerance, min_cluster_size, max_cluster_size);
        double time_taken = result.second;
        dbscan_times.push_back(time_taken);
    }
    
    double avg_dbscan_time = std::accumulate(dbscan_times.begin(), dbscan_times.end(), 0.0) / dbscan_times.size();
    std::cout << "\nDBSCAN Clustering (C++):" << std::endl;
    std::cout << "  Parameters: cluster_tolerance=" << cluster_tolerance 
              << ", min_cluster_size=" << min_cluster_size 
              << ", max_cluster_size=" << max_cluster_size << std::endl;
    std::cout << "  Average processing time: " << std::fixed << std::setprecision(6) << avg_dbscan_time << " seconds" << std::endl;
    
    return 0;
}
