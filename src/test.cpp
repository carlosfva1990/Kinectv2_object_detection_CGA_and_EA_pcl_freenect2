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
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/features/normal_3d.h>

#include <pcl/filters/extract_indices.h>

#include <type_traits>
#include "detail/vsr_multivector.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal PointNType;



double tress = 0.01;
double down = 0.025;
bool _downSample = true;
bool _resta = true;
bool _cluster = false;
bool _ciclo = true;
bool _ciclo2 = false;


boost::shared_ptr<pcl::PointCloud<PointType>> cloudKinect;
pcl::PointCloud<PointType>::Ptr cloudAux(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloudAux2(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloudCopy(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>);
pcl::NormalEstimation<PointType, PointNType > ne;
// Creating the KdTree object for the search method of the extraction
pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
std::vector<pcl::PointIndices> cluster_indices;


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


/*-----------------------------------------------------------------------------

a geometric algebra is constructed by first defining a bilinear form (i.e. a "metric tensor") over a field

*-----------------------------------------------------------------------------*/

//a euclidean algebra of 3d space (basis elements include vectors, areas, volumes, rotors (aka quaternions) )
//...................................<p>, field  >
using ega = vsr::algebra< vsr::metric<3>, double >;

//a metric "spacetime algebra" of 4d space 
//...................................<p,q>, field  >  ( p is number of basis roots of 1 and q number of roots of -1)
using sta = vsr::algebra< vsr::metric<1, 3>, double >;

//a conformal algebra of 3d space + 2 (we add two new dimensions to the "signature" and set conformal flag to "true")
//...................................<p,q,conf?>  field >
using cga = vsr::algebra< vsr::metric<4, 1, true>, double >;

//a twistor algebra of conformalized spacetime  
//...................................<p,q>, field  >
using twistor = vsr::algebra< vsr::metric<2, 4, true>, double >;




/*-----------------------------------------------------------------------------

the field over which the algebra is defined (in the above case "double")
can be any type of number that returns the same type under multiplication and addition
and is commutative, that is a*b = b*a (additionally it must have characteristic != 2 )

for instance, we can define a 2 dimensional geometric algebra over the complex numbers

*-----------------------------------------------------------------------------*/

//  a complex number is isomorphic to a "rotor" in 2 dimensional euclidean geometric algebra
template<class T> using complex_t = typename vsr::algebra< vsr::metric<2>, T >::types::rotor;

// c2 is now a four-dimensioal complex number in the tensored space i.e. with complex arguments
using c2 = complex_t< complex_t<double> >;


/*-----------------------------------------------------------------------------

template alias-driven "syntactic sugar" exists for common spaces such as euclidean and conformal.
in the first line of code above we could have written:

using ega = vsr::euclidean<3,double>;

or using the default field (i.e. "double") defined by the macro VSR_PRECISION:

using ega = vsr::euclidean<3>;

similarly,

using cga = vsr::conformal<5>;

Also, "::types::rotor" is syntactic sugar for ::make_sum< blade<2,0>::type, blade<2,2>::type >)
which generates a multivector type for the algebra

*-----------------------------------------------------------------------------*/
static_assert(std::is_same< ega, vsr::euclidean<3> >::value, "euclidean geometric algebra of 3D space");
static_assert(std::is_same< cga, vsr::conformal<5> >::value, "conformal geometric algebra of 3D space");

int main(int argc, char * argv[])
{








	static_assert(std::is_same<ega::types::vector::basis, typename vsr::blade<3, 1>::type >::value, "ega3d vector basis");
	static_assert(std::is_same<ega::types::bivector::basis, typename vsr::blade<3, 2>::type >::value, "ega3d bivector basis");


	printf("some euclidean 3D types\n");

	printf("vector\n");
	ega::types::vector::basis::print();
	printf("bivector\n");
	ega::types::bivector::basis::print();
	printf("trivector\n");
	ega::types::trivector::basis::print();
	printf("rotor\n");
	ega::types::rotor::basis::print();

	printf("some spacetime 4D types\n");

	printf("vector\n");
	sta::make_grade<1>::basis::print();

	printf("some combinatorics: product of rotors a and b in euclidean algebra: \n");
	ega::impl::gp_arrow_t< ega::types::rotor::basis, ega::types::rotor::basis >::Arrow::print();
	printf("some combinatorics: product of vectors a and b in space time algebra: \n");
	sta::impl::gp_arrow_t< sta::make_grade<1>::basis, sta::make_grade<1>::basis >::Arrow::print();
	printf("some combinatorics: product of vectors a and b in euclidean 4d algebra: \n");
	vsr::euclidean<4>::impl::gp_arrow_t< sta::make_grade<1>::basis, sta::make_grade<1>::basis >::Arrow::print();
	printf("some combinatorics: product of bivectors a and b in twistor algebra: \n");
	twistor::impl::gp_arrow_t< sta::make_grade<2>::basis, sta::make_grade<2>::basis >::Arrow::print();

	printf("some conformal 3D types\n");

	printf("point\n");
	cga::types::point::basis::print();
	printf("pair\n");
	cga::types::pair::basis::print();
	printf("circle\n");
	cga::types::circle::basis::print();
	printf("sphere\n");
	cga::types::sphere::basis::print();


	complex_t<double> ta(1.0, 2.5);
	complex_t<double> tb(2.0, 3.0);

	c2 a(ta, tb);
	//c2 b( ta, tb );//1.2, 3.2 );

	//  std::cout << ega::types::vector(1,2,3) << std::endl;
	a.print();

	(a*a).print();
























  //se define e inicia la captura de datos de kinect usando freenect y k2g
  Processor freenectprocessor = CUDA;//si no tienes cuda instalado prueba con OPENGL o CPU
  std::vector<int> ply_file_indices;
  K2G k2g(freenectprocessor);
  k2g.disableLog();
  
 

  std::cout << "getting cloud" << std::endl;
  cloudKinect = k2g.getCloud();
  //k2g.printParameters();

  cloudAux->sensor_orientation_.w() = 0.0;
  cloudAux->sensor_orientation_.x() = 1.0;
  cloudAux->sensor_orientation_.y() = 0.0;
  cloudAux->sensor_orientation_.z() = 0.0;

  cloudOut->sensor_orientation_.w() = 0.0;
  cloudOut->sensor_orientation_.x() = 1.0;
  cloudOut->sensor_orientation_.y() = 0.0;
  cloudOut->sensor_orientation_.z() = 0.0;

  //se crea una ventana y se vincula con el objeto viewer 
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Data Kinect"));
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer2(new pcl::visualization::PCLVisualizer("fig"));

  //se divide la ventana en 2 para mostrar la entrada y la salida
  //para el lado izq
  int v1(0);
  viewer2->setBackgroundColor(0, 0, 0);
  viewer->setPosition(350, 50);
  viewer2->setPosition(1240, 60);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10);


  viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
  viewer->setBackgroundColor(0, 0, 0, v1);
  viewer2->addText("Kinect data input", 10, 10, "v1 text");
  viewer2->addText("s: save cloud to ply", 10, 180, "v1 text2");
  viewer2->addText("p: copy to filter", 10, 170, "v1 text3");
  viewer2->addText("u: on/off filter", 10, 160, "v1 text4");
  viewer2->addText("j: increase threshold filter", 10, 150, "v1 text5");
  viewer2->addText("m: reduce  threshold filter", 10, 140, "v1 text6");
  viewer2->addText("y: on/off douwn sample", 10, 130, "v1 text7");
  viewer2->addText("h: increase threshold douwn sample", 10, 120, "v1 text8");
  viewer2->addText("n: reduce  threshold douwn sample", 10, 110, "v1 text9");
  viewer2->addText("o: pausa la ejecucion del programa", 10, 100, "v1 text10");
  viewer2->addText("l: ejecuta un ciclo del programa y se pausa", 10, 90, "v1 text11" );
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
  while(!viewer->wasStopped() && !viewer2->wasStopped()){

	  //permite la visualozacion de los datos en las ventanas
	  viewer->spinOnce();
	  viewer2->spinOnce(); 

	  //pausa la ejecucion
	  if (_ciclo)
	  {

		  //se obtienen los datos del kinect
		  k2g.updateCloud(cloudKinect);
		  //k2g.get(color, depth, cloudKinect);
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

		  if (_cluster && _resta)
		  {
			  double probSphere = 0;
			  double probPlane = 0;
			  double probCylinder = 0;

			  bool _sphere = false;
			  bool _plane = false;
			  bool _cylinder = false;

			  viewer2->removeAllPointClouds();
			  viewer2->removeAllShapes();
			  viewer2->addCoordinateSystem(1);
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
				  pcl::PointCloud<PointNType>::Ptr cloud_normals(new pcl::PointCloud<PointNType>);


				  //copia cada punto descrito por los inices
				  for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
					  cloud_cluster->points.push_back(cloudOut->points[*pit]); 

				  cloud_cluster->width = cloud_cluster->points.size();
				  cloud_cluster->height = 1;
				  cloud_cluster->is_dense = true;

				  std::vector<int> inliers;//indice de pintos que pertenecen a la figura


				  // se calcula RANSAC para la esfera
				  pcl::SampleConsensusModelSphere<PointType>::Ptr model_s(new pcl::SampleConsensusModelSphere<PointType>(cloud_cluster));
				  model_s->setRadiusLimits(0.05, 0.5);
				  pcl::RandomSampleConsensus<PointType> ransacS(model_s);
				  ransacS.setDistanceThreshold(.01);//tolerancia de error en la figura
				  ransacS.computeModel();
				  ransacS.getInliers(inliers);
				  probSphere = (double)inliers.size()/ cloud_cluster->width;

				  // se calcula RANSAC para el plano 
				  pcl::SampleConsensusModelPlane<PointType>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointType>(cloud_cluster));
				  pcl::RandomSampleConsensus<PointType> ransacP(model_p);
				  ransacP.setDistanceThreshold(.01);//tolerancia de error en la figura
				  ransacP.computeModel();
				  ransacP.getInliers(inliers);
				  probPlane = (double)inliers.size()/ cloud_cluster->width;

				  
				  // se calcula RANSAC para el cilindro 
				  pcl::SampleConsensusModelCylinder<PointType, PointNType >::Ptr model_c(new pcl::SampleConsensusModelCylinder<PointType, PointNType >(cloud_cluster));
				  //model_c =pcl::SampleConsensusModelCylinder<PointType, pcl::PointXYZRGBNormal>(cloud_cluster);
				  ne.setSearchMethod(tree);
				  ne.setInputCloud(cloud_cluster);
				  ne.setKSearch(50);
				  ne.compute(*cloud_normals);
				  model_c->setInputNormals(cloud_normals);
				  model_c->setInputCloud(cloud_cluster);
				  model_c->setRadiusLimits(0.02, 1);
				  pcl::RandomSampleConsensus<PointType> ransacC(model_c);
				  ransacC.setDistanceThreshold(.01);//tolerancia de error en la figura
				  ransacC.computeModel();
				  ransacC.getInliers(inliers);
				  probCylinder = (double)inliers.size() / cloud_cluster->width;



				  //se muestran las probabilidades para cada objeto
				  std::cout << "sphere prob "<< probSphere << std::endl;
				  std::cout << "plane prob " << probPlane << std::endl;
				  std::cout << "cylinder prob " << probCylinder << std::endl;

				  //se elige la mejor probabilidad
				  if (probSphere > probCylinder && probSphere > probPlane)
					  _sphere = true;
				  if (probCylinder > probPlane && probCylinder > probSphere)
					  _cylinder = true;
				  if (probPlane > probCylinder && probPlane > probSphere)
					  _plane = true;


				  if (_sphere)
				  {
					  ransacS.getInliers(inliers);
					  //se obtienen los datos de la figura y se convirten en un tipo de dato para graficar
					  Eigen::VectorXf coefficients;
					  ransacS.getModelCoefficients(coefficients);
					  pcl::PointXYZ center;
					  center.x = coefficients[0];
					  center.y = coefficients[1];
					  center.z = coefficients[2];
					  //se crea una esfera que correspunde al modelo calculado y se muestra
					  viewer2->addSphere(center, coefficients[3], 0.5, 0.0, 0.0, "sphere" + std::to_string(j));

					  std::cout << "sphere" + std::to_string(j) << std::endl;
					  _sphere = false;

				  }
				  if (_plane)
				  {

					  ransacP.getInliers(inliers);//calcula los puntos de la fugura
					  //se obtienen los datos de la figura y se convirten en un tipo de dato para graficar
					  pcl::ModelCoefficients coefficientsM;
					  Eigen::VectorXf coefficients;
					  coefficientsM.values.resize(4);
					  ransacP.getModelCoefficients(coefficients);
					  pcl::PointXYZ center;
					  coefficientsM.values[0] = coefficients[0];//centro x
					  coefficientsM.values[1] = coefficients[1];//centro y
					  coefficientsM.values[2] = coefficients[2];//centro z
					  coefficientsM.values[3] = coefficients[3];//radio
					  //se crea un plano que correspunde al modelo calculado y se muestra
					  viewer2->addPlane(coefficientsM, "plane" + std::to_string(j));

					  std::cout << "plane" + std::to_string(j) << std::endl;
					  _plane = false;
				  }
				  if (_cylinder)
				  {

					  ransacC.getInliers(inliers);
					  //se obtienen los datos de la figura y se convirten en un tipo de dato para graficar
					  pcl::ModelCoefficients coefficientsM;
					  Eigen::VectorXf coefficients;
					  coefficientsM.values.resize(7);
					  ransacC.getModelCoefficients(coefficients);
					  pcl::PointXYZ center;
					  coefficientsM.values[0] = coefficients[0];// centro x
					  coefficientsM.values[1] = coefficients[1];// centro y
					  coefficientsM.values[2] = coefficients[2];// centro z
					  coefficientsM.values[3] = coefficients[3];// direccion del eje x
					  coefficientsM.values[4] = coefficients[4];// direccion del eje y 
					  coefficientsM.values[5] = coefficients[5];// direccion del eje z
					  coefficientsM.values[6] = coefficients[6];// radio
					  viewer2->addCylinder(coefficientsM, "cylinder" + std::to_string(j));

					  std::cout << "cylinder" + std::to_string(j) << std::endl;
					  _cylinder = false;
				  }

				  //se copian solo los puntos de la figura(esfera, plano o cilindro) a otra nube de puntos
				  pcl::copyPointCloud<PointType>(*cloud_cluster, inliers, *cloud_cluster2);
				  //se agreega la nube resultante a la ventana para la visualizasion
				  viewer2->addPointCloud<PointType>(cloud_cluster2, "cloud" + std::to_string(j));
				  //se sima uno al contador de cluster
				  j++;
				  std::cout << "cluster" + std::to_string(j) << std::endl;
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