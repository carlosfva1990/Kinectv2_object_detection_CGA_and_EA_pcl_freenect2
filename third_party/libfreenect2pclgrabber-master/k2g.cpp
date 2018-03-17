#include "k2g.h"
#include <libfreenect2.hpp>
#include <frame_listener_impl.h>
#include <packet_pipeline.h>
#include <registration.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <Eigen/Core>

bool stop = false;


enum processor{
	CPU, OPENCL, OPENGL
};

void sigint_handler(int s)
{
        stop = true;
}



        K2G::K2G(bool mirror): mirror_(mirror), listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth),
                                               undistorted_(512, 424, 4), registered_(512, 424, 4), big_mat_(1920, 1082, 4), qnan_(std::numeric_limits<float>::quiet_NaN()){

                signal(SIGINT,sigint_handler);

		if(freenect2_.enumerateDevices() == 0)
		{
			std::cout << "no kinect2 connected!" << std::endl;
			exit(-1);
		}

		processor p = OPENGL;

		serial_ = freenect2_.getDefaultDeviceSerialNumber();
		switch(p){
			case CPU:
				std::cout << "creating CPU processor" << std::endl;
				pipeline_ = new libfreenect2::CpuPacketPipeline();
				break;
#ifdef HAVE_OPENCL
			case OPENCL:
				std::cout << "creating OpenCL processor" << std::endl;
				pipeline_ = new libfreenect2::OpenCLPacketPipeline();
				break;
#endif
			case OPENGL:
				std::cout << "creating OpenGL processor" << std::endl;
				pipeline_ = new libfreenect2::OpenGLPacketPipeline();
				break;
			default:
				std::cout << "creating CPU processor" << std::endl;
				pipeline_ = new libfreenect2::CpuPacketPipeline();
				break;
		}
		
		dev_ = freenect2_.openDevice(serial_, pipeline_);
		dev_->setColorFrameListener(&listener_);
		dev_->setIrAndDepthFrameListener(&listener_);
		dev_->start();

		registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

		prepareMake3D(dev_->getIrCameraParams());
 	}

	libfreenect2::Freenect2Device::IrCameraParams K2G::getIrParameters(){
		libfreenect2::Freenect2Device::IrCameraParams ir = dev_->getIrCameraParams();
		return ir;
	}

	libfreenect2::Freenect2Device::ColorCameraParams K2G::getRgbParameters(){
		libfreenect2::Freenect2Device::ColorCameraParams rgb = dev_->getColorCameraParams();
		return rgb;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr K2G::getCloud(){
		const short w = undistorted_.width;
		const short h = undistorted_.height;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(w, h));
        
		return updateCloud(cloud);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr K2G::updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
		
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];

		registration_->apply(rgb, depth, &undistorted_, &registered_, true, &big_mat_, map_);
		const std::size_t w = undistorted_.width;
		const std::size_t h = undistorted_.height;

        cv::Mat tmp_itD0(undistorted_.height, undistorted_.width, CV_8UC4, undistorted_.data);
        cv::Mat tmp_itRGB0(registered_.height, registered_.width, CV_8UC4, registered_.data);
        
        if (mirror_ == true){

            cv::flip(tmp_itD0,tmp_itD0,1);
            cv::flip(tmp_itRGB0,tmp_itRGB0,1);

        }

        const float * itD0 = (float *) tmp_itD0.ptr();
        const char * itRGB0 = (char *) tmp_itRGB0.ptr();
        
		pcl::PointXYZRGB * itP = &cloud->points[0];
        bool is_dense = true;
		
		for(std::size_t y = 0; y < h; ++y){

			const unsigned int offset = y * w;
			const float * itD = itD0 + offset;
			const char * itRGB = itRGB0 + offset * 4;
			const float dy = rowmap(y);

			for(std::size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4 )
			{
				const float depth_value = *itD / 1000.0f;
				
				if(!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)){
	
					const float rx = colmap(x) * depth_value;
                	const float ry = dy * depth_value;               
					itP->z = depth_value;
					itP->x = rx;
					itP->y = ry;

					itP->b = itRGB[0];
					itP->g = itRGB[1];
					itP->r = itRGB[2];
				} else {
					itP->z = qnan_;
					itP->x = qnan_;
					itP->y = qnan_;

					itP->b = qnan_;
					itP->g = qnan_;
					itP->r = qnan_;
					is_dense = false;
 				}
			}
		}
		cloud->is_dense = is_dense;
		listener_.release(frames_);
		return cloud;
	}

	void K2G::shutDown(){
		dev_->stop();
  		dev_->close();
	}

	cv::Mat K2G::getColor(){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		cv::Mat tmp(rgb->height, rgb->width, CV_8UC4, rgb->data);
		cv::Mat r;
        if (mirror_ == true) {cv::flip(tmp,r,1);}
        else {r = tmp.clone();}
        
		listener_.release(frames_);
		return std::move(r);
	}

	cv::Mat K2G::getDepth(){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];
		cv::Mat tmp(depth->height, depth->width, CV_8UC4, depth->data);
		cv::Mat r;
        if (mirror_ == true) {cv::flip(tmp,r,1);}
        else {r = tmp.clone();}

		listener_.release(frames_);
		return std::move(r);
	}

        std::pair<cv::Mat, cv::Mat> K2G::getDepthRgb(){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		registration_->apply(rgb, depth, &undistorted_, &registered_);
		cv::Mat tmp_depth(undistorted_.height, undistorted_.width, CV_8UC4, undistorted_.data);
		cv::Mat tmp_color(registered_.height, registered_.width, CV_8UC4, registered_.data);
		cv::Mat r = tmp_color.clone();
		cv::Mat d = tmp_depth.clone();
        if (mirror_ == true) {
            cv::flip(tmp_depth,d,1);
            cv::flip(tmp_color,r,1);
        }
		listener_.release(frames_);
		return std::move(std::pair<cv::Mat, cv::Mat>(r,d));
	}


	void K2G::prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p)
	{
		const int w = 512;
		const int h = 424;
	    float * pm1 = colmap.data();
	    float * pm2 = rowmap.data();
	    for(int i = 0; i < w; i++)
	    {
	        *pm1++ = (i-depth_p.cx + 0.5) / depth_p.fx;
	    }
	    for (int i = 0; i < h; i++)
	    {
	        *pm2++ = (i-depth_p.cy + 0.5) / depth_p.fy;
	    }
	}

