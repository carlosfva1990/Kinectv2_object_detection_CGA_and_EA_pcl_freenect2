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
pcl::PointCloud<PointType>::Ptr cloudAux2(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloudCopy(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>);

int PrimeraVez = 0;
double tress = 0.01;
double down = 0.025;
bool _downSample = true;
bool _resta = true;
bool _ciclo = true;

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
		pcl::copyPointCloud(*cloudAux2, *cloudCopy);
		std::cout << "cpiado" << std::endl;
	}
	if (pressed == "o")
	{ 
		//sirbe para debugear y ver que ocurre en cada ciclo
		//hay que descomentar una linea en main que hace _ciclo = false
		_ciclo = true;
		std::cout << "ciclo" << std::endl;
	}
	if (pressed == "u")
	{
		if (_resta)
			_resta = false;
		else
			_resta = true;
		std::cout << "resta " << _resta << std::endl;
	}
	if (pressed == "j")
	{
		tress = tress + 0.001;
		std::cout << "tres " << tress << std::endl;
	}
	if (pressed == "m")
	{
		tress = tress - 0.001;
		std::cout << "tres " << tress << std::endl;
	}
	if (pressed == "y")
	{
		if (_downSample)
			_downSample = false;
		else
			_downSample = true;
		std::cout << "downSample " << _downSample << std::endl;
	}
	if (pressed == "h")
	{
		down = down + 0.001;
		std::cout << "down " << down << std::endl;
	}
	if (pressed == "n")
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
  std::cout << "Press \'s\' to store a cloud" << std::endl;
  std::cout << "Press \'x\' to store the calibrations." << std::endl;

  //se define e inicia la captura de datos de kinect usando freenect y k2g
  Processor freenectprocessor = OPENGL;
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

  //se crea una ventana y se vincula con el objeto viewer 
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

  //se divide la ventana en 2 para mostrar la entrada y la salida
  //para el lado izq
  int v1(0);
  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addText("Kinect data input",10,10, "v1 text", v1);
  viewer->addPointCloud<PointType>(cloudAux, "sample cloud", v1);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  //para el lado der
  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
  viewer->addText("Filtered data",10,10, "v2 text", v2);
  viewer->addPointCloud<PointType>(cloudOut, "sample cloud2", v2);

  // para usar menos puntos de la nuve
  pcl::VoxelGrid<PointType> vg;

  // para realizar la dferencia entre 2 nuves de puntos
  pcl::SegmentDifferences<PointType> resta;

  //se agrega un evento para el teclado para poder guardar la nuve
  PlySaver ps(cloudAux2, false, false, k2g);
  viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);

  //mat de cv2 obtiene las imagenes del kinect
  cv::Mat color, depth;
  
  //para reducir la cantidad de puntos
  vg.setInputCloud(cloudAux);
  vg.setLeafSize(down, down, down);


  //para eliminar los puntos que no se usan
  resta.setInputCloud(cloudAux2);
  resta.setTargetCloud(cloudCopy);
  resta.setDistanceThreshold(tress);

  //inicia el ciclo con los datos ya iniciados
  while(!viewer->wasStopped()){

	  if (_ciclo)
	  {
		  //limpia la ventana delos datos anteriores
		  viewer->spinOnce();

		  //se obtienen los datos del kinect
		  k2g.get(color, depth, cloudKinect);

		  copyPointCloud(*cloudKinect, *cloudAux);

		  if (_downSample)
		  {
			  //se reducen la cantidad de puntos
			  vg.setLeafSize(down, down, down);
			  vg.filter(*cloudAux2);
		  }
		  else
		  {
			  copyPointCloud(*cloudAux, *cloudAux2);
		  }

		  if (_resta)
		  {
			  //se eliminan los puntos que no se usan
			  resta.segment(*cloudOut);
		  }
		  else
		  {
			  copyPointCloud(*cloudAux2, *cloudOut);
		  }

		  //se envian los datos de las nuves de puntos y mostrarlos en la ventana
		  viewer->updatePointCloud<PointType>(cloudAux, "sample cloud");
		  viewer->updatePointCloud(cloudOut, "sample cloud2");

		  //_ciclo = false;
	  }
  }

  k2g.shutDown();
  return 0;
}