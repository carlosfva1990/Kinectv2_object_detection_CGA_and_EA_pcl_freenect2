/*
Copyright 2015, Giacomo Dabisias"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@Author 
Giacomo Dabisias, PhD Student
PERCRO, (Laboratory of Perceptual Robotics)
Scuola Superiore Sant'Anna
via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
*/

#include <libfreenect2.hpp>
#include <frame_listener_impl.h>
#include <packet_pipeline.h>
#include <registration.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

#include <cstdlib>
#include <string>
#include <iostream>
#include <Eigen/Core>

extern bool stop;


extern void sigint_handler(int s);

class K2G {

public:

        K2G(bool mirror = false);

	libfreenect2::Freenect2Device::IrCameraParams getIrParameters();

	libfreenect2::Freenect2Device::ColorCameraParams getRgbParameters();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud();

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);

	void shutDown();

	cv::Mat getColor();

	cv::Mat getDepth();

	std::pair<cv::Mat, cv::Mat> getDepthRgb();
private:

	void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p);

	libfreenect2::Freenect2 freenect2_;
	libfreenect2::Freenect2Device * dev_ = 0;
	libfreenect2::PacketPipeline * pipeline_ = 0;
	libfreenect2::Registration * registration_ = 0;
	libfreenect2::SyncMultiFrameListener listener_;
	libfreenect2::FrameMap frames_;
	libfreenect2::Frame undistorted_, registered_, big_mat_;
	Eigen::Matrix<float,512,1> colmap;
	Eigen::Matrix<float,424,1> rowmap;
	std::string serial_;
	int map_[512 * 424]; 
	float qnan_; 
	bool mirror_;  
};
