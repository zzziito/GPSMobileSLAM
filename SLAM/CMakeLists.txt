cmake_minimum_required(VERSION 2.8.3)
project(gps_offroad_slam)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
IF(COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/gps_offroad_slam/cmake_modules)

find_package (catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
cv_bridge
image_transport
tf
tf2_geometry_msgs
tf2_ros
sensor_msgs
dynamic_reconfigure
message_generation
)

find_package(OpenCV 4.0 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      find_package(OpenCV 2.4.3 QUIET)
      if(NOT OpenCV_FOUND)
         message(FATAL_ERROR "OpenCV > 2.4.3 not found.")
     endif()
   endif()
endif()

find_package(Eigen3 3.1.0 REQUIRED)

set (DYNAMIC_RECONFIGURE_PATH ros/config/dynamic_reconfigure.cfg)
execute_process(COMMAND chmod a+x ${DYNAMIC_RECONFIGURE_PATH}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
        RESULT_VARIABLE cmd_result
        OUTPUT_VARIABLE cmd_ver)
message(STATUS "Chmod a+x the dynamic_reconfigure file")

generate_dynamic_reconfigure_options(
        ${DYNAMIC_RECONFIGURE_PATH}
)

set(LIBS_GPSOFFSLAM
${OpenCV_LIBS}
${EIGEN3_LIBS}
${PROJECT_SOURCE_DIR}/gps_offroad_slam/Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/gps_offroad_slam/Thirdparty/g2o/lib/libg2o.so
)

set(LIBS_ROS
${PROJECT_SOURCE_DIR}/gps_offroad_slam/lib/lib${PROJECT_NAME}.so
${OpenCV_LIBS}
${catkin_LIBRARIES}
)

add_service_files(
        FILES
        SaveMap.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package (
  CATKIN_DEPENDS roscpp rospy std_msgs cv_bridge image_transport tf sensor_msgs dynamic_reconfigure message_runtime
  LIBRARIES {PROJECT_NAME} libDBoW2 libg2o
)

include_directories(
${PROJECT_SOURCE_DIR}/gps_offroad_slam
${PROJECT_SOURCE_DIR}/gps_offroad_slam/include
${PROJECT_SOURCE_DIR}/ros/include
${EIGEN3_INCLUDE_DIR}
${catkin_INCLUDE_DIRS}
)

include(${PROJECT_SOURCE_DIR}/gps_offroad_slam/Thirdparty/DBoW2/CMakeLists.txt)
include(${PROJECT_SOURCE_DIR}/gps_offroad_slam/Thirdparty/g2o/CMakeLists.txt)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/gps_offroad_slam/lib)

add_library(${PROJECT_NAME} SHARED
gps_offroad_slam/src/System.cc
gps_offroad_slam/src/Tracking.cc
gps_offroad_slam/src/LocalMapping.cc
gps_offroad_slam/src/LoopClosing.cc
gps_offroad_slam/src/ORBextractor.cc
gps_offroad_slam/src/ORBmatcher.cc
gps_offroad_slam/src/FrameDrawer.cc
gps_offroad_slam/src/Converter.cc
gps_offroad_slam/src/MapPoint.cc
gps_offroad_slam/src/KeyFrame.cc
gps_offroad_slam/src/Map.cc
gps_offroad_slam/src/Optimizer.cc
gps_offroad_slam/src/PnPsolver.cc
gps_offroad_slam/src/Frame.cc
gps_offroad_slam/src/KeyFrameDatabase.cc
gps_offroad_slam/src/Sim3Solver.cc
gps_offroad_slam/src/Initializer.cc
)
add_dependencies (${PROJECT_NAME} g2o DBoW2)

target_link_libraries(${PROJECT_NAME}
${LIBS_GPSOFFSLAM}
)

# map serialization addition - library boost serialization 
message(STATUS "Compile With map save/load function")
find_library(BOOST_SERIALIZATION boost_serialization)
if (NOT BOOST_SERIALIZATION)
    message(FATAL_ERROR "Can't find libboost_serialization")
endif()
target_link_libraries(${PROJECT_NAME} ${BOOST_SERIALIZATION})

add_executable (${PROJECT_NAME}_mono
ros/src/MonoNode.cc
ros/src/Node.cc
)
add_dependencies (${PROJECT_NAME}_mono ${PROJECT_NAME} ${PROJECT_NAME}_gencfg)

target_link_libraries(${PROJECT_NAME}_mono
${LIBS_ROS}
)


install(TARGETS ${PROJECT_NAME}_mono 
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME} DBoW2 g2o
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY ros/launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/launch
)

install(DIRECTORY ros/config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/ros/config
)

install(DIRECTORY gps_offroad_slam/config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/gps_offroad_slam/config
)

install(DIRECTORY gps_offroad_slam/Vocabulary/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/gps_offroad_slam/Vocabulary
)

