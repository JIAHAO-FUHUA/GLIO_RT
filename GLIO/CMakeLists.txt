cmake_minimum_required(VERSION 2.8.3)
project(GLIO)

#set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  pcl_ros
  pcl_conversions
  nlosExclusion
  gnss_comm
  rtcm_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(GTSAM REQUIRED)

message(STATUS "Using catkin version ${catkin_VERSION}")
message(STATUS "Using Ceres version ${CERES_VERSION}")

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp message_runtime std_msgs nav_msgs geometry_msgs
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
)

link_directories(
    include
    ${GTSAM_LIBRARY_DIRS}
)

include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(Preprocessing src/Preprocessing.cpp)
target_link_libraries(Preprocessing ${catkin_LIBRARIES})

add_executable(LidarOdometry src/LidarOdometry.cpp)
target_link_libraries(LidarOdometry ${catkin_LIBRARIES} ${CERES_LIBRARIES})

add_executable(Estimator src/Estimator.cpp src/MarginalizationFactor.cpp)
target_link_libraries(Estimator ${catkin_LIBRARIES} ${CERES_LIBRARIES} gtsam pcl_visualization)

#add_executable(FormatConvert src/FormatConvert.cpp)
#target_link_libraries(FormatConvert ${catkin_LIBRARIES})

add_executable(UbloxRos src/ubloxros.cpp)
# target_link_libraries(UbloxRos ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES} ${GTSAM_LIBRARIES})
# target_link_libraries(UbloxRos ${catkin_LIBRARIES} rtkcmn)
# target_link_libraries(UbloxRos ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES}
# 						convkml1 convrnx1 datum1 download1 ephemeris1 geoid1 ionex1 lambda1
# 						options1 pntpos1 postpos1 ppp_ar1 ppp1 preceph1 qzslex1 rcvraw1 rinex1
# 						rtcm1 rtcm21 rtcm31 rtcm3e1 rtkcmn1 rtkpos1 rtksvr1 sbas1 solution1
# 						stream1 streamsvr1 tle1
# 						)
target_link_libraries(UbloxRos ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${CERES_LIBRARIES}
        convkml convrnx datum download ephemeris geoid ionex lambda
        options pntpos postpos ppp_ar ppp preceph qzslex rcvraw rinex
        rtcm rtcm2 rtcm3 rtcm3e rtkcmn rtkpos rtksvr sbas solution
        stream streamsvr tle
        libGeographiccc)

add_definitions(-w)