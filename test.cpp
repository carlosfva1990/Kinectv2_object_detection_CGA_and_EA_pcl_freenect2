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
Giacomo  Dabisias, PhD Student
PERCRO, (Laboratory of Perceptual Robotics)
Scuola Superiore Sant'Anna
via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
*/
#include "k2g.h"
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>
#include <pcl/segmentation/segment_differences.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>

typedef pcl::PointXYZRGB PointType;

boost::shared_ptr<pcl::PointCloud<PointType>> cloudKinect;
pcl::PointCloud<PointType>::Ptr cloudAux(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloudCopy(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>);

int PrimeraVez = 0;
double tress = 0.01;
double down = 0.05;

struct PlySaver{


  PlySaver(boost::shared_ptr<pcl::PointCloud<PointType>> cloud, bool binary, bool use_camera, K2G & k2g):
           cloud_(cloud), binary_(binary), use_camera_(use_camera), k2g_(k2g){}

  boost::shared_ptr<pcl::PointCloud<PointType>> cloud_;
  bool binary_;
  bool use_camera_;
  K2G & k2g_;
};

//entrada por teclado
void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * data)
{
	
  std::string pressed = event.getKeySym();//lee los datos de la interrupcion      
  PlySaver * s = (PlySaver*)data;//para cuardar la nube de puntos
  if(event.keyDown ())
  {
    if(pressed == "s")
    {
      
      pcl::PLYWriter writer;
      std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
      std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
      writer.write ("cloud_" + now, *(s->cloud_), s->binary_, s->use_camera_);
      
      std::cout << "saved " << "cloud_" + now + ".ply" << std::endl;
    }
	if(pressed == "p")
	{
		//se guarda una imagen inicial en cloud2 para luego compararla
		pcl::copyPointCloud(*cloudAux, *cloudCopy);
		std::cout << "cpiado" << std::endl;
	}
	if (pressed == "o")
	{
		tress = tress + 0.001;
		std::cout << "tres " << tress << std::endl;
	}
	if (pressed == "l")
	{
		tress = tress - 0.001;
		std::cout << "tres " << tress << std::endl;
	}
	if (pressed == "i")
	{
		down = down + 0.001;
		std::cout << "down " << down << std::endl;
	}
	if (pressed == "k")
	{
		down = down - 0.001;
		std::cout << "down " << down << std::endl;
	}
    if(pressed == "x")
    {
        s->k2g_.storeParameters();
        std::cout << "stored calibration parameters" << std::endl;
    }
  }
}

int main(int argc, char * argv[])
{
  std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2,3 correspond to CPU, OPENCL, OPENGL, CUDA respectively\n";
  std::cout << "Press \'s\' to store a cloud" << std::endl;
  std::cout << "Press \'x\' to store the calibrations." << std::endl;

  Processor freenectprocessor = CPU;
  std::vector<int> ply_file_indices;
  K2G k2g(freenectprocessor);
  std::cout << "getting cloud" << std::endl;
  cloudKinect = k2g.getCloud();

  k2g.printParameters();

  cloudAux->sensor_orientation_.w() = 0.0;
  cloudAux->sensor_orientation_.x() = 1.0;
  cloudAux->sensor_orientation_.y() = 0.0;
  cloudAux->sensor_orientation_.z() = 0.0;

  cloudOut->sensor_orientation_.w() = 0.0;
  cloudOut->sensor_orientation_.x() = 1.0;
  cloudOut->sensor_orientation_.y() = 0.0;
  cloudOut->sensor_orientation_.z() = 0.0;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addText("Kinect data input",10,10, "v1 text", v1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloudAux);
  viewer->addPointCloud<PointType>(cloudAux, rgb, "sample cloud", v1);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
  viewer->addText("Filtered data",10,10, "v2 text", v2);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloudOut, "sample cloud2", v2);



  pcl::SegmentDifferences<PointType> resta;
  // Downsample to voxel grid
  pcl::VoxelGrid<PointType> vg;

  PlySaver ps(cloudKinect, false, false, k2g);
  viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);

  cv::Mat color, depth;
  
  while(!viewer->wasStopped()){

	viewer->spinOnce();
	k2g.get(color, depth, cloudKinect);
	vg.setInputCloud(cloudKinect);

	vg.setLeafSize(down, down, down);
	vg.filter(*cloudAux);
	
	resta.setInputCloud(cloudAux);
	resta.setTargetCloud(cloudCopy);
	resta.setDistanceThreshold(tress);
	resta.segment(*cloudOut);
	
    pcl::visualization::PointCloudColorHandlerRGBField<PointType> rgb(cloudAux);
	viewer->updatePointCloud<PointType>(cloudAux, rgb, "sample cloud");
	viewer->updatePointCloud(cloudOut, "sample cloud2");
  }
  k2g.shutDown();
  return 0;
}