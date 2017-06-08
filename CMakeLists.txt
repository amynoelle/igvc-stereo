#SET(execName usma_ros_zed)
SET(execName2 usma_ros_pcl)

##CMAKE_MINIMUM_REQUIRED(VERSION 2.4)
cmake_minimum_required(VERSION 2.8.3)
project(igvc_stereo)

if(COMMAND cmake_policy)
	cmake_policy(SET CMP0003 OLD)
	cmake_policy(SET CMP0015 OLD)
endif(COMMAND cmake_policy)

SET(VERSION_REQ_OCV "3.1")
#SET(VERSION_REQ_OCV "2.4")
SET(SPECIAL_OS_LIBS "")
SET(VERSION_REQ_CUDA "8.0")


##For Jetson, OpenCV4Tegra is based on OpenCV2.4
exec_program(uname ARGS -p OUTPUT_VARIABLE CMAKE_SYSTEM_NAME2)
if ( CMAKE_SYSTEM_NAME2 MATCHES "aarch64" OR CMAKE_SYSTEM_NAME2 MATCHES "armv7l" )
   SET(VERSION_REQ_OCV "2.4")
   SET(VERSION_REQ_CUDA "8.0")

   unset(CUDA_USE_STATIC_CUDA_RUNTIME CACHE)
   option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
   message ("CUDA_USE_STATIC_CUDA_RUNTIME : ${CUDA_USE_STATIC_CUDA_RUNTIME}")
   ##to prevent from opencv_dep_cudart dependencies error...
   ## cmake with -DCUDA_USE_STATIC_CUDA_RUNTIME=false can also be called.

endif()

set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} -arch compute_35 -code sm_35)
 
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rosconsole
  sensor_msgs
  dynamic_reconfigure
  std_msgs
  image_transport
  dynamic_reconfigure
  cv_bridge 
  pcl_conversions
  tf2_ros
)

find_package(OpenCV ${VERSION_REQ_OCV} COMPONENTS core highgui imgproc REQUIRED)
find_package(CUDA ${VERSION_REQ_CUDA} REQUIRED)
find_package(PCL)
find_package(ZED 2.0 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/color_filter.cfg
  #...
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rosconsole
    sensor_msgs
    opencv
    image_transport
    dynamic_reconfigure
    tf2_ros
    pcl_conversions
)
###############################################################################
# INCLUDES

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${CUDA_INCLUDE_DIRS}
	${ZED_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

link_directories(
    ${PCL_LIBRARY_DIRS}
    ${ZED_LIBRARY_DIR}
    ${OpenCV_LIBRARY_DIRS}
    ${CUDA_LIBRARY_DIRS}
)

###############################################################################
ADD_EXECUTABLE(${execName2} src/tx2main_pcl.cpp src/color_filter.cpp)
add_definitions(-std=c++0x -g -O3)

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4") ## needed if you install pcl from repo (and libproj-dev)

MESSAGE("OpenCV_INCLUDE_DIRS:      " ${OpenCV_INCLUDE_DIRS} )
MESSAGE("OpenCV_LIBRARIES:         " ${OpenCV_LIBRARIES} )

list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

TARGET_LINK_LIBRARIES(${execName2}
                        ${OpenCV_LIBRARIES}
                        ${catkin_LIBRARIES}
                        ${ZED_LIBRARIES}
                        ${CUDA_LIBRARIES} ${CUDA_npps_LIBRARY} ${CUDA_nppi_LIBRARY}
                        ${PCL_LIBRARIES}
                    )
                    
                    
                    
                    
                    
