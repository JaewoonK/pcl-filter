cmake_minimum_required(VERSION 3.5)
project(pcl_filtering)

# Set C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find PCL
find_package(PCL 1.8 REQUIRED)

# Include PCL directories
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Include project headers
include_directories(include)

# Add executables
add_executable(pcl_filtering src/main.cpp)
add_executable(create_test_cloud src/create_test_cloud.cpp)
add_executable(visualize_clouds src/visualize_clouds.cpp)

# Link against PCL libraries
target_link_libraries(pcl_filtering ${PCL_LIBRARIES})
target_link_libraries(create_test_cloud ${PCL_LIBRARIES})
target_link_libraries(visualize_clouds ${PCL_LIBRARIES})

# Unit tests with Google Test
option(BUILD_TESTS "Build the test programs" ON)

if(BUILD_TESTS)
  # Enable testing
  enable_testing()
  
  # Find GTest
  find_package(GTest)
  
  if(GTEST_FOUND)
    # Add test executable
    add_executable(filter_test src/filter_test.cpp)
    
    # Link against PCL and GTest
    target_link_libraries(filter_test ${PCL_LIBRARIES} ${GTEST_BOTH_LIBRARIES} pthread)
    
    # Add test to CTest
    add_test(NAME FilterTest COMMAND filter_test)
  else()
    message(STATUS "Google Test not found, tests will not be built")
  endif()
endif()
