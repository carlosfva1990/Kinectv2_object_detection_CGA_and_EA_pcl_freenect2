# Kinectv2_pcl_freenect2_SegmentDifferences

this is an example project where using the sensor kinect v2 the point cloud is represented using  pcl 1.8,
and for grabbing the data is a libfreenect2 grabber.
 
if you want the original grabber project the link is:
 
https://github.com/giacomodabisias/libfreenect2pclgrabber
===

## Installation 

first of all i used a i7 processor with 8gb of ram and windows 10 pro

the software i used:

*[Cmake 3.10.1](https://cmake.org/download/) (to link and create the msvc project)
*[Microsoft visual studio 2017 community](https://www.visualstudio.com/es/downloads/?rr=https%3A%2F%2Fwww.google.com.mx%2F) (c++ editor and compiler)
*[Kinect SDK v2.0_1409](https://www.microsoft.com/en-us/download/details.aspx?id=44561)
*[PCL 1.8.1 (all in one)](http://unanancyowen.com/en/pcl18/)
*[VTK 5.8.0 Qt suport](http://sourceforge.net/projects/pointclouds/files/dependencies/VTK-5.8.0-msvc2010-win64.exe/download)
*[OpenCV 3.4.0](https://sourceforge.net/projects/opencvlibrary/files/opencv-win/)
*[libfreenect2](https://github.com/OpenKinect/libfreenect2)
*[OpenNI2](https://github.com/OpenNI/OpenNI2) (or 
			https://s3.amazonaws.com/com.occipital.openni/OpenNI-Windows-x64-2.2.0.33.zip)


when i build **libfreenect2** i needed to copy ´"config.h"´ and "export.h" from /build_dir/libfreenect2/ to /include/libfreenect2/
and i copy the dll files to c:/windows/system32/ to avoid copying the files in every project folder.

