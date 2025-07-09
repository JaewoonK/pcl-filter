#include <gtest/gtest.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <random>

class StatisticalOutlierRemovalTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Create point cloud with known outliers
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>());
        
        // Set basic parameters
        cloud->width = 1050;
        cloud->height = 1;
        cloud->is_dense = false;
        cloud->points.resize(cloud->width * cloud->height);
        
        // Random number generators with fixed seed for reproducibility
        std::mt19937 gen(42); // Fixed seed
        std::uniform_real_distribution<> normal_dist(-0.5, 0.5);
        std::uniform_real_distribution<> outlier_dist(-1.5, 1.5);
        
        // Generate 1000 normal points
        for (size_t i = 0; i < 1000; ++i) {
            cloud->points[i].x = normal_dist(gen);
            cloud->points[i].y = normal_dist(gen);
            cloud->points[i].z = normal_dist(gen);
        }
        
        // Generate 50 outlier points (much farther from center)
        for (size_t i = 1000; i < 1050; ++i) {
            cloud->points[i].x = outlier_dist(gen);
            cloud->points[i].y = outlier_dist(gen);
            cloud->points[i].z = outlier_dist(gen);
            
            // Track outlier points for verification
            outlier_indices.push_back(i);
        }
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
    std::vector<size_t> outlier_indices;
};

TEST_F(StatisticalOutlierRemovalTest, RemovesOutliers) {
    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);      // 50 nearest neighbors
    sor.setStddevMulThresh(1.0); // Standard deviation threshold
    
    // Apply the filter
    sor.filter(*cloud_filtered);
    
    // Verify that the filtered cloud has fewer points than the original
    EXPECT_LT(cloud_filtered->size(), cloud->size());
    
    // Count how many of our known outliers were removed
    int outliers_removed = 0;
    for (size_t idx : outlier_indices) {
        bool found = false;
        
        // Check if this outlier point exists in the filtered cloud
        for (const auto& filtered_point : cloud_filtered->points) {
            if (std::abs(filtered_point.x - cloud->points[idx].x) < 1e-5 &&
                std::abs(filtered_point.y - cloud->points[idx].y) < 1e-5 &&
                std::abs(filtered_point.z - cloud->points[idx].z) < 1e-5) {
                found = true;
                break;
            }
        }
        
        if (!found) {
            outliers_removed++;
        }
    }
    
    // We expect a significant number of our known outliers to be removed
    // Not necessarily all 50, but a good portion of them
    EXPECT_GT(outliers_removed, 30);
    
    std::cout << "Original cloud size: " << cloud->size() << std::endl;
    std::cout << "Filtered cloud size: " << cloud_filtered->size() << std::endl;
    std::cout << "Points removed: " << (cloud->size() - cloud_filtered->size()) << std::endl;
    std::cout << "Known outliers removed: " << outliers_removed << " out of " << outlier_indices.size() << std::endl;
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
