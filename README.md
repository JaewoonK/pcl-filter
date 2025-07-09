# Point Cloud Filtering with PCL

This project demonstrates and compares two different point cloud filtering methods using the Point Cloud Library (PCL):

1. **Statistical Outlier Removal (SOR)** - Removes outliers based on statistical analysis of point neighborhoods
2. **DBSCAN Clustering** - Identifies clusters and marks low-density regions as outliers

## Minimal Workflow

The essential components for the workflow are:

1. **create_test_cloud.py** - Generates synthetic point cloud data with intentional outliers
2. **visualize_clouds.cpp** - Filters the point cloud using both methods and visualizes the results

## How the Filters Work

### Statistical Outlier Removal (SOR)

SOR works by analyzing the statistical distribution of point-to-neighbor distances:

1. For each point, it finds the k nearest neighbors (k = meanK parameter)
2. Calculates the average distance to those neighbors
3. Computes the mean (μ) and standard deviation (σ) of all these average distances
4. Points whose average distance falls outside μ ± (stddev_mult * σ) are classified as outliers and removed

**Parameters:**
- `meanK`: Number of nearest neighbors to analyze (higher values consider more points)
- `stddev_mult`: Standard deviation multiplier threshold (lower values remove more points)

### DBSCAN Clustering

DBSCAN (Density-Based Spatial Clustering of Applications with Noise) works by grouping points based on density:

1. Groups points that are closely packed together (points with many neighbors)
2. Marks points in low-density regions as outliers (noise)
3. Identifies clusters based on density connectivity

**Parameters:**
- `cluster_tolerance`: Maximum distance between points to be considered in the same cluster
- `min_cluster_size`: Minimum number of points required to form a cluster
- `max_cluster_size`: Maximum number of points allowed in a cluster

## Prerequisites

- PCL 1.8 or higher
- CMake 3.5 or higher
- C++11 compatible compiler
- Python with Open3D (for test cloud generation)

## Building the Project

```bash
mkdir -p build
cd build
cmake ..
make
```

## Usage

### 1. Generate a Test Point Cloud

```bash
python create_test_cloud.py
```

This creates an `input_cloud.pcd` file with 1000 normal points and 50 outlier points.

### 2. Filter and Visualize

```bash
cd build
./visualize_clouds
```

This will:
1. Load the point cloud from `input_cloud.pcd`
2. Apply both filtering methods (SOR and DBSCAN)
3. Display a three-viewport visualization:
   - Left: Original point cloud (white)
   - Middle: Statistical Outlier Removal result (green)
   - Right: DBSCAN clustering result (blue)
4. Save the filtered results as `filtered_cloud_sor.pcd` and `filtered_cloud_dbscan.pcd`

You can also specify a different input file:

```bash
./visualize_clouds path/to/your_cloud.pcd
```

## Adjusting Filter Parameters

To experiment with different filter parameters, modify the following lines in `src/visualize_clouds.cpp`:

### For Statistical Outlier Removal:
```cpp
sor.setMeanK(50);              // Number of nearest neighbors
sor.setStddevMulThresh(1.0);    // Standard deviation multiplier
```

### For DBSCAN:
```cpp
cloud_filtered_dbscan = applyDBSCAN(cloud, 0.15, 10, 10000);
// Parameters: cloud, cluster_tolerance, min_cluster_size, max_cluster_size
```

## Additional Files

The project also includes some supplementary files that are not essential for the main workflow:

- **main.cpp** - A simpler version that only applies Statistical Outlier Removal without visualization
- **create_test_cloud.cpp** - C++ alternative to create_test_cloud.py
- **filter_test.cpp** - Unit tests for the Statistical Outlier Removal filter

## License

This project is licensed under the MIT License - see the LICENSE file for details.
