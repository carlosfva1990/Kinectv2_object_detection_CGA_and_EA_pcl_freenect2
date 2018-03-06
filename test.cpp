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
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>

#include <pcl/filters/extract_indices.h>

typedef pcl::PointXYZRGB PointType;

boost::shared_ptr<pcl::PointCloud<PointType>> cloudKinect;
pcl::PointCloud<PointType>::Ptr cloudAux(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloudAux2(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloudCopy(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>);

// Creating the KdTree object for the search method of the extraction
pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
std::vector<pcl::PointIndices> cluster_indices;

int PrimeraVez = 0;
double tress = 0.01;
double down = 0.025;
bool _downSample = true;
bool _resta = true;
bool _ciclo = true;
bool _ciclo2 = false;
bool _cluster = false;

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
		tree->setInputCloud(cloudOut);
		_cluster = true;
		std::cout << "cpiado" << std::endl;
	}
	if (pressed == "o")
	{
		//sirbe para debugear y ver que ocurre en cada ciclo
		//hay que descomentar una linea en main que hace _ciclo = false
		if (_ciclo)
			_ciclo = false;
		else
			_ciclo = true;

		std::cout << "ciclo" << std::endl;
	}
	if (pressed == "l")
	{
		//sirbe para debugear y ver que ocurre en cada ciclo
		//hay que descomentar una linea en main que hace _ciclo = false
		_ciclo = true;
		_ciclo2 = true;

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


void PreProsessing(pcl::visualization::PCLVisualizer& viewer)
{

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
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("3D Viewer"));

  //se divide la ventana en 2 para mostrar la entrada y la salida
  //para el lado izq
  int v1(0);
  viewer2->setBackgroundColor(0, 0, 0);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10);


  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer->addText("Kinect data input", 10, 10, "v1 text", v1);
  viewer->addText("s: save cloud to ply", 10, 180, "v1 text2", v1);
  viewer->addText("p: copy to filter", 10, 170, "v1 text3", v1);
  viewer->addText("u: on/off filter", 10, 160, "v1 text4", v1);
  viewer->addText("j: increase threshold filter", 10, 150, "v1 text5", v1);
  viewer->addText("m: reduce  threshold filter", 10, 140, "v1 text6", v1);
  viewer->addText("y: on/off douwn sample", 10, 130, "v1 text7", v1);
  viewer->addText("h: increase threshold douwn sample", 10, 120, "v1 text8", v1);
  viewer->addText("n: reduce  threshold douwn sample", 10, 110, "v1 text9", v1);
  viewer->addText("o: pausa la ejecucion del programa", 10, 100, "v1 text10", v1);
  viewer->addText("l: ejecuta un ciclo del programa y se pausa", 10, 90, "v1 text11", v1);
  viewer->addPointCloud<PointType>(cloudAux, "sample cloud", v1);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5);
  //para el lado der
  int v2(0);
  viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
  viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
  viewer->addText("Filtered data",10,10, "v2 text", v2); 
  viewer->addPointCloud<PointType>(cloudOut, "sample cloud2", v2);

  //se agrega un evento para el teclado para poder guardar la nuve
  PlySaver ps(cloudAux2, false, false, k2g);
  viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);
  viewer2->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);

  // para usar menos puntos de la nuve
  pcl::VoxelGrid<PointType> vg;

  // para realizar la dferencia entre 2 nuves de puntos
  pcl::SegmentDifferences<PointType> resta;

  //mat de cv2 obtiene las imagenes del kinect
  cv::Mat color, depth;
  
  //para reducir la cantidad de puntos
  vg.setInputCloud(cloudAux);
  vg.setLeafSize(down, down, down);


  //para eliminar los puntos que no se usan (filtro kalman)
  resta.setInputCloud(cloudAux2);
  resta.setTargetCloud(cloudCopy);
  resta.setDistanceThreshold(tress);

  //clusterizacion de objetos usando la distancia euclidiana
  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance(down); 
  ec.setMinClusterSize(50);
  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloudOut);
  ec.extract(cluster_indices);


  //inicia el ciclo con los datos ya iniciados
  while(!viewer->wasStopped()|| !viewer2->wasStopped()){

	  //permite la visualozacion de los datos en las ventanas
	  viewer->spinOnce();
	  viewer2->spinOnce(); 

	  //pausa la ejecucion
	  if (_ciclo)
	  {

		  //se obtienen los datos del kinect
		  k2g.get(color, depth, cloudKinect);
		  //se copian a una nube auxiliar
		  copyPointCloud(*cloudKinect, *cloudAux);

		  if (_downSample)
		  {
			  //se reducen la cantidad de puntos
			  vg.setLeafSize(down, down, down);
			  ec.setClusterTolerance(down+0.01); //la toleracia para el cluster es poco mas grande que la de la reduccion 
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

		  if (_cluster)
		  {
			  //se obtienen los fiferentes objetos en diferentes nuves de puntos
			  int j = 0;//contador de clusters
			  std::cout << "cluster" << std::endl;
			  std::vector<pcl::PointIndices> cluster_indices;//hay que reiniciarlos indices
			  ec.extract(cluster_indices);
			  //para cada nube de puntos(objetos) encontrada:
			  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
			  {
				  pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>);
				  pcl::PointCloud<PointType>::Ptr cloud_cluster2(new pcl::PointCloud<PointType>);
				  //copia cada punto descrito por los inices
				  for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
					  cloud_cluster->points.push_back(cloudOut->points[*pit]); 

				  cloud_cluster->width = cloud_cluster->points.size();
				  cloud_cluster->height = 1;
				  cloud_cluster->is_dense = true;


				  std::vector<int> inliers;//indice de pintos que pertenecen a la figura

				  // created RandomSampleConsensus object and compute the appropriated model
				  pcl::SampleConsensusModelSphere<PointType>::Ptr model_s(new pcl::SampleConsensusModelSphere<PointType>(cloud_cluster));
				  pcl::RandomSampleConsensus<PointType> ransac(model_s);
				  ransac.setDistanceThreshold(.01);//tolerancia de error en la figura
				  ransac.computeModel();
				  ransac.getInliers(inliers);

				  //se copian solo los puntos de la figura(esfera, plano o cilindro) a otra nube de puntos
				  pcl::copyPointCloud<PointType>(*cloud_cluster, inliers, *cloud_cluster2);

				  cloud_cluster2->sensor_orientation_.w() = 0.0;
				  cloud_cluster2->sensor_orientation_.x() = 1.0;
				  cloud_cluster2->sensor_orientation_.y() = 0.0;
				  cloud_cluster2->sensor_orientation_.z() = 0.0;

				  //se agreega la nube resultante a la ventana para la visualizasion
				  if (! viewer2->updatePointCloud(cloud_cluster2, "cloud" + std::to_string(j)))
				  {
					  viewer2->addPointCloud<PointType>(cloud_cluster2, "cloud" + std::to_string(j));
					  std::cout << "cluster" + std::to_string(j) << std::endl;
				  }
				  
				  j++;
			  }
		  }


		  //se envian los datos de las nuves de puntos y mostrarlos en la ventana
		  viewer->updatePointCloud(cloudOut, "sample cloud2");
		  viewer->updatePointCloud<PointType>(cloudAux, "sample cloud");
		 // boost::this_thread::sleep(boost::posix_time::milliseconds(1000));

		  //si se usa la tecla de ciclo unico
		  if(_ciclo2)
			_ciclo = false;
	  }
  }

  k2g.shutDown();
  return 0;
}