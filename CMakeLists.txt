cmake_minimum_required(VERSION 3.0.2)
project(TBLIO)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        sensor_msgs
        std_msgs
        tf
        tf_conversions
        pcl_conversions
        pcl_ros
        )

find_package(GTSAM REQUIRED)

find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
find_package(Ceres REQUIRED PATHS "/home/xcy/catkin_ws/install_isolated/lib/cmake/Ceres")



catkin_package(
        LIBRARIES dbscan3d LO
        CATKIN_DEPENDS roscpp sensor_msgs std_msgs tf tf_conversions
)

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
)
include_directories("/usr/include/eigen3")
include_directories(${CERES_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${GTSAM_INCLUDE_DIR})


add_executable(TBLIO_node src/TBLIO_node.cpp src/TBLIO.cpp src/TBLIO.h)
target_link_libraries(TBLIO_node ${Boost_LIBRARIES} -lgtsam -ltbb)
target_link_libraries(TBLIO_node
        ${catkin_LIBRARIES}
        )

#add_executable(dbscan3d src/dbscan3d.cpp src/kd_tree_nn.cpp src/kd_tree_nn.h src/octree_nn.cpp src/octree_nn.h src/kmeans.cpp src/kmeans.h src/gmm_EM.cpp src/gmm_EM.h src/spectral_clustering.cpp src/spectral_clustering.h src/dbscan_clustering.cpp src/dbscan_clustering.h)
ADD_EXECUTABLE(TBLIO_SLAM src/dbscan3d.cpp
        src/octree_nn.cpp src/octree_nn.h
        src/groundRemovalRANSAC.cpp src/groundRemovalRANSAC.h
        src/dbscan_correction.cpp src/dbscan_correction.h)
target_link_libraries( TBLIO_SLAM ${CERES_LIBRARIES} )
add_dependencies(TBLIO_SLAM ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(TBLIO_SLAM ${catkin_LIBRARIES})

add_library(LO src/treeCenterLocalization.cpp src/octree_nn.cpp src/octree_nn.h)
target_link_libraries( LO ${CERES_LIBRARIES} )
target_link_libraries( LO dlib::dlib )

add_executable(LO_node src/treeCenterLocalization_node.cpp)
target_link_libraries(LO_node treeCenterLocalization)
add_dependencies(LO ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(LO ${catkin_LIBRARIES})

