# Point Cloud Filtering Project

This project demonstrates point cloud filtering using PCL (Point Cloud Library) in C++. It applies statistical outlier removal to clean up noisy point cloud data.

## Prerequisites

- CMake (version 3.5 or higher)
- PCL (Point Cloud Library, version 1.8 or higher)

## Building the Project

```bash
# Navigate to the build directory
cd build

# Generate build files
cmake ..

# Build the project
make
```

## Running the Application

The application expects an input PCD file named `input_cloud.pcd` in the current directory. It will process this file and output a filtered point cloud as `filtered_cloud.pcd`.

```bash
# From the build directory
./pcl_filtering
```

## Creating a Sample Point Cloud

If you don't have a PCD file for testing, you can create one using the following Python script (requires numpy and open3d):

```python
import numpy as np
import open3d as o3d

# Create a point cloud with some outliers
points = np.random.rand(1000, 3)  # 1000 points in 3D space
# Add some outliers
outliers = np.random.rand(50, 3) * 2  # 50 outlier points
all_points = np.vstack([points, outliers])

# Create Open3D point cloud
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(all_points)

# Save to PCD file
o3d.io.write_point_cloud("input_cloud.pcd", pcd)
print("Created input_cloud.pcd with 1050 points (including 50 outliers)")
```

## Unit Testing

To test the filtering functionality, you can create a unit test that:
1. Generates a known point cloud with outliers
2. Applies the statistical outlier removal filter
3. Verifies that the outliers have been removed

This can be implemented using a testing framework like Google Test.
