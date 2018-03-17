# Copyright 2015, Giacomo Dabisias"
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.
#
# @Author 
# Giacomo Dabisias, PhD Student
# PERCRO, (Laboratory of Perceptual Robotics)
# Scuola Superiore Sant'Anna
# via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy

cmake_minimum_required(VERSION 2.8)
Project(kinect2grabber CXX)

set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -std=c++11") 
set(CMAKE_CXX_FLAGS   "${CMAKE_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE  Release)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/modules")
set(INCLUDE           "${CMAKE_SOURCE_DIR}/include")

option(WITH_OPENCL "adds opencl support for freenect2" OFF)
if(${WITH_OPENCL})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_OPENCL")
endif()

option(WITH_CUDA "adds cuda support for freenect2" OFF)
if(${WITH_CUDA})
    SET(LIBFREENECT2_WITH_CUDA_SUPPORT 1)
    SET(HAVE_CUDA yes)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_CUDA")
endif()

option(WITH_SERIALIZATION "adds support for boost serialization" OFF)
if(${WITH_SERIALIZATION})
	find_package(Boost REQUIRED)
	include_directories(${INCLUDE})
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_SERIALIZATION")
	set(BOOST_LIBS "boost_serialization")
endif()

option(WITH_ROS "adds support for ROS" OFF)
if(${WITH_ROS})
	find_package(catkin REQUIRED COMPONENTS   roscpp std_msgs pcl_conversions  pcl_ros
						sensor_msgs cv_bridge image_transport tf
						message_generation )
	catkin_package( CATKIN_DEPENDS roscpp rospy std_msgs pcl_conversions  pcl_ros
						sensor_msgs cv_bridge image_transport tf
						cmake_modules
	)
	include_directories(${catkin_INCLUDE_DIRS} include)
endif()


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DWITH_PCL")
find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${PCL_INCLUDE_DIRS})

find_package(OpenCV     REQUIRED)
find_package(Freenect2  REQUIRED)
find_package (VTK REQUIRED)



include_directories (
		include
        third_party/versor/include
        third_party/versor/include/vsr
        third_party/libfreenect2pclgrabber-master
		${PCL_INCLUDE_DIRS}
		${FREENECT2_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
)


link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

add_library(versor
        third_party/versor/src/space/vsr_cga3D_op.cpp
        third_party/versor/src/space/vsr_cga3D_frame.cpp
        third_party/versor/src/space/vsr_cga3D_round.cpp
        test.cpp
		src/objects_body.cpp
        src/ransac_body.cpp
        #src/window/pclviewer.cpp
	third_party/libfreenect2pclgrabber-master/k2g.cpp
	)





add_executable(Kinect2Grabber     
		test.cpp
		src/objects_body.cpp
        src/ransac_body.cpp)

target_link_libraries(Kinect2Grabber      ${OpenCV_LIBS} ${FREENECT2_LIBRARY} ${PCL_LIBRARIES} ${BOOST_LIBS} versor ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${FREENECT2_LIBRARY} ${OpenCV_LIBS} ${VTK_LIBRARIES})

