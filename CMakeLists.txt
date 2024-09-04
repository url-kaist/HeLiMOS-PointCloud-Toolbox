cmake_minimum_required(VERSION 3.5)
project(helipr_tool)

# Set the C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Find required packages
find_package(PCL 1.8 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP QUIET)
find_package(yaml-cpp REQUIRED) 

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR} 
  include
)

# Add definitions and directories from dependencies
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# Compiler flags
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

# Set the PCL directory (adjust as necessary)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl")

# Source files
set(SAVER_FILES src/pointSaver.cpp src/visualizer.cpp src/PointCloudProcessor.cpp src/utility.cpp src/BsplineSE3.cpp)
set(MERGER_FILES src/pointMerger.cpp src/visualizer.cpp src/PointCloudProcessor.cpp src/utility.cpp src/BsplineSE3.cpp)
set(PROPAGATE_FILES src/pointPropagator.cpp src/visualizer.cpp src/PointCloudProcessor.cpp src/utility.cpp src/BsplineSE3.cpp)
# Add executable
add_executable(helimos_saver ${SAVER_FILES})

# Link libraries
target_link_libraries(helimos_saver 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${EIGEN3_LIBRARIES}
  ${YAML_CPP_LIBRARIES} 
)

add_executable(helimos_merger ${MERGER_FILES})

# Link libraries
target_link_libraries(helimos_merger 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${EIGEN3_LIBRARIES}
  ${YAML_CPP_LIBRARIES} 
)

add_executable(helimos_propagator ${PROPAGATE_FILES})

target_link_libraries(helimos_propagator 
  ${catkin_LIBRARIES} 
  ${PCL_LIBRARIES} 
  ${EIGEN3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  
)
