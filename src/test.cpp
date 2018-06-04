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
#include "ransac.h"
#include <time.h>

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal PointNType;

double tress = 0.01;
double down = 0.025;
bool _downSample = true;
bool _resta = true;
bool _cluster = false;
bool _ciclo = true;
bool _ciclo2 = false;
bool _useAgc = true;
bool _debug = true;

boost::shared_ptr<pcl::PointCloud<PointType>> cloudKinect;
pcl::PointCloud<PointType>::Ptr cloudAux(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloudAux2(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloudCopy(new pcl::PointCloud<PointType>);
pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>);
// Creating the KdTree object for the search method of the extraction
pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);


struct PlySaver {


	PlySaver(boost::shared_ptr<pcl::PointCloud<PointType>> cloud, bool binary, bool use_camera, K2G & k2g) :
		cloud_(cloud), binary_(binary), use_camera_(use_camera), k2g_(k2g) {}

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
	if (event.keyDown())
	{
		if (pressed == "a")
		{
			if (_useAgc)
				_useAgc = false;
			else
				_useAgc = true;
			std::cout << "use Agc " << _useAgc << std::endl;
		}
		if (pressed == "p")
		{
			//se guarda una imagen inicial en cloud2 para luego compararla
			pcl::copyPointCloud(*cloudAux2, *cloudCopy);
			tree->setInputCloud(cloudOut);
			_cluster = true;
			_downSample = false;
			std::cout << "cpiado" << std::endl;
		}
		if (pressed == "s")
		{

			pcl::PLYWriter writer;
			std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
			std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
			writer.write("cloud_" + now, *(s->cloud_), s->binary_, s->use_camera_);

			std::cout << "saved " << "cloud_" + now + ".ply" << std::endl;
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
		if (pressed == "x")
		{
			s->k2g_.storeParameters();
			std::cout << "stored calibration parameters" << std::endl;
		}
	}
}



int main(int argc, char * argv[])
{

	//se define e inicia la captura de datos de kinect usando freenect y k2g
	Processor freenectprocessor = CUDA;//si no tienes cuda instalado prueba con OPENGL o CPU
	std::vector<int> ply_file_indices;
	K2G k2g(freenectprocessor);
	k2g.disableLog();

	k2g.printParameters();
	std::cout << "getting cloud" << std::endl;
	cloudKinect = k2g.getCloudMed();

	//se crea una ventana y se vincula con el objeto viewer 
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Data Kinect"));

	//se divide la ventana en 2 para mostrar la entrada y la salida
	//para el lado izq
	int v1(0);
	viewer->setPosition(350, 50);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10);
	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0, 0, 0, v1);
	viewer->addPointCloud<PointType>(cloudAux, "sample cloud", v1);

	//para el lado der
	int v2(0);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.1, 0.1, 0.1, v2);
	viewer->addText("Filtered data", 10, 10, "v2 text", v2);
	viewer->addPointCloud<PointType>(cloudOut, "sample cloud2", v2);

	//se agrega un evento para el teclado para poder guardar la nuve
	PlySaver ps(cloudAux2, false, false, k2g);
	viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);

	//se acomoda la camara para ver los datos
	viewer->setCameraPosition(1, 0, -4, 0, -1, 0, 0);

	//inicia el ciclo con los datos ya iniciados
	while (!viewer->wasStopped()) {
		
		clock_t timeStart = clock();

		//se envian los datos de los nuvos puntos y se muestran en la ventana
		viewer->updatePointCloud(cloudOut, "sample cloud2");
		viewer->updatePointCloud<PointType>(cloudAux, "sample cloud");

		//permite la visualizacion de los datos 
		viewer->spinOnce(2, true);

		//pausa la ejecucion
		if (_ciclo)
		{
			//se obtienen los datos del kinect
			k2g.updateCloudMed(cloudKinect);

			//se copian a una nube auxiliar
			copyPointCloud(*cloudKinect, *cloudAux);

			//se limpian las figuras del visor
			viewer->removeAllShapes();
			//se re-agrega el menu
			viewer->addText("Kinect data input", 10, 10, "v1 text", v1);
			viewer->addText("a: change between AGC an vector metods", 10, 180, "v1 text12", v1);
			viewer->addText("p: copy to filter", 10, 180, "v1 text3", v1);
			viewer->addText("s: save cloud to ply", 10, 170, "v1 text2", v1);
			viewer->addText("o: pausa la ejecucion del programa", 10, 160, "v1 text10", v1);
			viewer->addText("l: ejecuta un ciclo del programa y se pausa", 10, 150, "v1 text11", v1);
			viewer->addText("u: on/off filter", 10, 140, "v1 text4", v1);
			viewer->addText("j: increase threshold filter", 10, 130, "v1 text5", v1);
			viewer->addText("m: reduce  threshold filter", 10, 120, "v1 text6", v1);
			viewer->addText("y: on/off douwn sample", 10, 110, "v1 text7", v1);
			viewer->addText("h: increase threshold douwn sample", 10, 100, "v1 text8", v1);
			viewer->addText("n: reduce  threshold douwn sample", 10, 90, "v1 text9", v1);

			if (_debug) {
				std::cout << "tiempo de actualizacion de los puntos:  " << (double)(clock() - timeStart) / CLOCKS_PER_SEC << std::endl;
				timeStart = clock();
			}
			//se realiza una disminucion de puntos
			if (_downSample)
			{
				//vg es el objeto que realiza la disminucion
				pcl::VoxelGrid<PointType> vg;
				vg.setInputCloud(cloudAux);
				vg.setLeafSize(down, down, down);
				vg.filter(*cloudAux2);//el resultado se guarda en cloudAux2
			}
			else
			{
				copyPointCloud(*cloudAux, *cloudAux2);//el resultado se guarda en cloudAux2
			}

			if (_debug) {
				std::cout << "tiempo disminucion de puntos:  " << (double)(clock() - timeStart) / CLOCKS_PER_SEC << std::endl;
				timeStart = clock();
			}
			// para realizar la dferencia entre 2 nuves de puntos
			//para eliminar los puntos que no se usan 
			if (_resta)
			{
				//la resta se realiza cloudAux-cloudCopy con una toleracia tress
				pcl::SegmentDifferences<PointType> resta;
				resta.setInputCloud(cloudAux2);
				resta.setTargetCloud(cloudCopy);
				resta.setDistanceThreshold(tress);
				resta.segment(*cloudOut);//la resta se guarda en cloudOut
			}
			else
			{
				copyPointCloud(*cloudAux2, *cloudOut);
			}

			if (_debug) {
				std::cout << "tiempo dresta de las nubes:  " << (double)(clock() - timeStart) / CLOCKS_PER_SEC << std::endl;
				timeStart = clock();
			}

			//clusterizacion de objetos usando la distancia euclidiana
			if (_cluster && _resta)
			{
				pcl::EuclideanClusterExtraction<PointType> ec;
				ec.setClusterTolerance(down + 0.01); //la toleracia para el cluster es poco mas grande que la de la reduccion 
				ec.setMinClusterSize(50);
				ec.setMaxClusterSize(25000);
				ec.setSearchMethod(tree);
				ec.setInputCloud(cloudOut);

				//se obtienen los diferentes objetos en diferentes nuves de puntos
				int j = 0;//contador de clusters
				std::vector<pcl::PointIndices> cluster_indices;//arreglo que contiene los incices de puntos de cada cluster
				ec.extract(cluster_indices);

				//para cada nube de puntos(objetos) encontrada:
				for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
				{
					pcl::PointCloud<PointType>::Ptr cloud_cluster(new pcl::PointCloud<PointType>);//nube de puntos (cluster)
				   // pcl::PointCloud<PointType>::Ptr cloud_cluster2(new pcl::PointCloud<PointType>);
					pcl::PointCloud<PointNType>::Ptr cloud_normals(new pcl::PointCloud<PointNType>);//normales calculadas de la nuve de puntos

					//copia cada punto descrito por los inices
					for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
						cloud_cluster->points.push_back(cloudOut->points[*pit]);

					cloud_cluster->width = cloud_cluster->points.size();
					cloud_cluster->height = 1;
					cloud_cluster->is_dense = true;

					//se calcula el mejor modelo para cada cluster
					if (cloud_cluster->width > 20) //el cluster deve tener almenos 20 puntos
					{
						double probSphere = 0;
						double probPlane = 0;
						double probCylinder = 0;

						bool _sphere = false;
						bool _plane = false;
						bool _cylinder = false;

						std::vector<int> inliers;//indice de pintos que pertenecen a la figura

						if (_debug) {
							timeStart = clock();
						}
						//usando el metodo tradicional
						if (!_useAgc)
						{
							///////////////////////////////////////////////////////////////////////////////////////////////////////
							////////////////////////////////////////modelo de la esfera ///////////////////////////////////////////
							///////////////////////////////////////////////////////////////////////////////////////////////////////
							pcl::SampleConsensusModelSphere<PointType>::Ptr model_s(new pcl::SampleConsensusModelSphere<PointType>(cloud_cluster));
							model_s->setRadiusLimits(0.05, 0.5);
							pcl::RandomSampleConsensus<PointType> ransacS(model_s);
							// se calcula RANSAC para la esfera
							ransacS.setDistanceThreshold(.01);//tolerancia de error en la figura
							ransacS.computeModel();
							ransacS.getInliers(inliers);
							probSphere = (double)inliers.size() / cloud_cluster->width;

							if (_debug) {
								std::cout << "tiempo para estimar la esfera:  " << (double)(clock() - timeStart) / CLOCKS_PER_SEC << std::endl;
								timeStart = clock();
							}

							///////////////////////////////////////////////////////////////////////////////////////////////////////
							/////////////////////////////////////////modelo del plano /////////////////////////////////////////////
							///////////////////////////////////////////////////////////////////////////////////////////////////////
							pcl::SampleConsensusModelPlane<PointType>::Ptr model_p(new pcl::SampleConsensusModelPlane<PointType>(cloud_cluster));
							pcl::RandomSampleConsensus<PointType> ransacP(model_p);
							// se calcula RANSAC para el plano 
							ransacP.setDistanceThreshold(.01);//tolerancia de error en la figura
							ransacP.computeModel();
							ransacP.getInliers(inliers);
							probPlane = (double)inliers.size() / cloud_cluster->width;

							if (_debug) {
								std::cout << "tiempo para estimar el plano:  " << (double)(clock() - timeStart) / CLOCKS_PER_SEC << std::endl;
								timeStart = clock();
							}

							///////////////////////////////////////////////////////////////////////////////////////////////////////
							//////////////////////////////////////////// cilindro /////////////////////////////////////////////////
							///////////////////////////////////////////////////////////////////////////////////////////////////////
							//calculo de normales
							pcl::NormalEstimation<PointType, PointNType > ne;
							ne.setSearchMethod(tree);
							ne.setInputCloud(cloud_cluster);
							ne.setKSearch(50);
							ne.compute(*cloud_normals);
							//modelo del cilindro
							pcl::SampleConsensusModelCylinder<PointType, PointNType >::Ptr model_c(new pcl::SampleConsensusModelCylinder<PointType, PointNType >(cloud_cluster));
							model_c->setInputNormals(cloud_normals);
							model_c->setInputCloud(cloud_cluster);
							model_c->setRadiusLimits(0.02, 1);
							pcl::RandomSampleConsensus<PointType> ransacC(model_c);
							// se calcula RANSAC para el cilindro 
							ransacC.setDistanceThreshold(.01);//tolerancia de error en la figura
							ransacC.computeModel();
							ransacC.getInliers(inliers);
							probCylinder = (double)inliers.size() / cloud_cluster->width;


							if (_debug) {
								std::cout << "tiempo para estimar el cilindo:  " << (double)(clock() - timeStart) / CLOCKS_PER_SEC << std::endl;
								timeStart = clock();
							}

							/*
							//se muestran las probabilidades para cada objeto
							std::cout << "sphere prob " << probSphere << std::endl;
							std::cout << "plane prob " << probPlane << std::endl;
							std::cout << "cylinder prob " << probCylinder << std::endl;
							*/
							//se elige la mejor probabilidad
							if (probSphere > probCylinder && probSphere > probPlane)
								_sphere = true;
							if (probCylinder > probPlane && probCylinder > probSphere)
								_cylinder = true;
							if (probPlane > probCylinder && probPlane > probSphere)
								_plane = true;

							if (_sphere)
							{
								//ransacS.getInliers(inliers);

								//se obtienen los datos de la figura y se convirten en un tipo de dato para graficar
								Eigen::VectorXf coefficients;
								ransacS.getModelCoefficients(coefficients);
								pcl::PointXYZ center;
								center.x = coefficients[0];
								center.y = coefficients[1];
								center.z = coefficients[2];

								//se crea una esfera que correspunde al modelo calculado y se muestra
								viewer->addSphere(center, coefficients[3], 0.5, 0.0, 0.0, "sphere" + std::to_string(j), 0);
								std::cout << "sphere" + std::to_string(j) << std::endl;
								_sphere = false;
							}

							if (_plane)
							{

								//ransacP.getInliers(inliers);//calcula los puntos de la fugura
															//se obtienen los datos de la figura y se convirten en un tipo de dato para graficar
								pcl::ModelCoefficients coefficientsM;
								Eigen::VectorXf coefficients;
								coefficientsM.values.resize(4);
								ransacP.getModelCoefficients(coefficients);
								coefficientsM.values[0] = coefficients[0];//x
								coefficientsM.values[1] = coefficients[1];//y
								coefficientsM.values[2] = coefficients[2];//z
								coefficientsM.values[3] = coefficients[3];//w//d Hessian component
																		  //se crea un plano que correspunde al modelo calculado y se muestra
								viewer->addPlane(coefficientsM, "plane" + std::to_string(j), 0);

								std::cout << "plane" + std::to_string(j) << std::endl;
								_plane = false;
							}

							if (_cylinder)
							{
								//ransacC.getInliers(inliers);
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
								viewer->addCylinder(coefficientsM, "cylinder" + std::to_string(j), 0);
								std::cout << "cylinder" + std::to_string(j) << std::endl;
								_cylinder = false;
							}

						}
						//usando algebra geometrica
						else
						{

							if (_debug) {
								timeStart = clock();
							}

							///////////////////////////////////////////////////////////////////////////////////////////////////////
							////////////////////////////////////////modelo de la esfera ///////////////////////////////////////////
							///////////////////////////////////////////////////////////////////////////////////////////////////////
							//se calcula ransac para la esfera en agc
							ransacSphere sph;
							sph.setData(cloud_cluster, 0.001);
							sph.compute();
							inliers = *sph.indexlist;
							probSphere = (double)inliers.size() / cloud_cluster->width;

							if (_debug) {
								std::cout << "tiempo para estimar la esfera:  " << (double)(clock() - timeStart) / CLOCKS_PER_SEC << std::endl;
								timeStart = clock();
							}


							///////////////////////////////////////////////////////////////////////////////////////////////////////
							/////////////////////////////////////////modelo del plano /////////////////////////////////////////////
							///////////////////////////////////////////////////////////////////////////////////////////////////////
							//se calcula ransac para el plano en agc
							ransacPlane pln;
							pln.setData(cloud_cluster, 0.003, 1);
							pln.compute();
							inliers = *pln.indexlist;
							probPlane = (double)inliers.size() / cloud_cluster->width;

							if (_debug) {
								std::cout << "tiempo para estimar el plano:  " << (double)(clock() - timeStart) / CLOCKS_PER_SEC << std::endl;
								timeStart = clock();
							}

							///////////////////////////////////////////////////////////////////////////////////////////////////////
							//////////////////////////////////////////// cilindro /////////////////////////////////////////////////
							///////////////////////////////////////////////////////////////////////////////////////////////////////
							//se calcula ransac para el cilindro en agc
							ransacCylinder2 cyl;
							cyl.setData(cloud_cluster, 0.0071);
							cyl.compute();
							inliers = *cyl.indexlist;
							probCylinder = (double)inliers.size() / cloud_cluster->width;

							if (_debug) {
								std::cout << "tiempo para estimar el cilindro:  " << (double)(clock() - timeStart) / CLOCKS_PER_SEC << std::endl;
								timeStart = clock();
							}

							/*
							//se muestran las probabilidades para cada objeto
							std::cout << "sphere prob " << probSphere << std::endl;
							std::cout << "plane prob " << probPlane << std::endl;
							std::cout << "cylinder prob " << probCylinder << std::endl;
							*/
							//se elige la mejor probabilidad
							if (probSphere > probCylinder && probSphere > probPlane)
								_sphere = true;
							if (probCylinder > probPlane && probCylinder > probSphere)
								_cylinder = true;
							if (probPlane > probCylinder && probPlane > probSphere)
								_plane = true;

							if (_sphere) {
								//inliers = *sph.indexlist;
								pcl::PointXYZ center;
								center.x = sph.ranSph[0];
								center.y = sph.ranSph[1];
								center.z = sph.ranSph[2];
								//se crea una esfera que correspunde al modelo calculado y se muestra
								viewer->addSphere(center, sph.rad, 0.5, 0.0, 0.0, "sphere" + std::to_string(j), 0);

								std::cout << "sphere" + std::to_string(j) << std::endl;
								_sphere = false;

							}

							if (_plane)
							{
								//inliers = *pln.indexlist;
								pcl::PointXYZ center;
								pcl::ModelCoefficients coefficientsM;
								coefficientsM.values.resize(4);
								coefficientsM.values[0] = pln.ranPln[0];//x normal
								coefficientsM.values[1] = pln.ranPln[1];//y normal
								coefficientsM.values[2] = pln.ranPln[2];//z normal
								coefficientsM.values[3] = -1 * pln.ranPln[4];//d Hessian component

								//se crea una esfera que correspunde al modelo calculado y se muestra
								viewer->addPlane(coefficientsM, "plane" + std::to_string(j), 0);

								std::cout << "plane" + std::to_string(j) << std::endl;
								_plane = false;
							}

							if (_cylinder)
							{
								//inliers = *cyl.indexlist;
								//se obtienen los datos de la figura y se convirten en un tipo de dato para graficar
								pcl::ModelCoefficients coefficientsM;
								Eigen::VectorXf coefficients;
								coefficientsM.values.resize(7);
								pcl::PointXYZ center;
								coefficientsM.values[0] = cyl.ranCyl.center[0];// centro x
								coefficientsM.values[1] = cyl.ranCyl.center[1];// centro y
								coefficientsM.values[2] = cyl.ranCyl.center[2];// centro z
								coefficientsM.values[3] = cyl.ranCyl.plan[0];// direccion del eje x
								coefficientsM.values[4] = cyl.ranCyl.plan[1];// direccion del eje y 
								coefficientsM.values[5] = cyl.ranCyl.plan[2];// direccion del eje z
								coefficientsM.values[6] = cyl.ranCyl.radius;// radio
								viewer->addCylinder(coefficientsM, "cylinder" + std::to_string(j), 0);

								pcl::PointXYZ center1;
								center1.x = cyl.ranCyl.s1.x;
								center1.y = cyl.ranCyl.s1.y;
								center1.z = cyl.ranCyl.s1.z;
								//se crea una esfera que correspunde al modelo calculado y se muestra
								viewer->addSphere(center1, cyl.ranCyl.s1.radius, 0.5, 0.0, 0.0, "sphere1" + std::to_string(j), 0);

								pcl::PointXYZ center2;
								center2.x = cyl.ranCyl.s2.x;
								center2.y = cyl.ranCyl.s2.y;
								center2.z = cyl.ranCyl.s2.z;
								//se crea una esfera que correspunde al modelo calculado y se muestra
								viewer->addSphere(center2, cyl.ranCyl.s2.radius, 0.5, 0.0, 0.0, "sphere2" + std::to_string(j), 0);

								std::cout << "cylinder" + std::to_string(j) << std::endl;
								_cylinder = false;
							}

							//se copian solo los puntos de la figura(esfera, plano o cilindro) a otra nube de puntos
						   // pcl::copyPointCloud<PointType>(*cloud_cluster, inliers, *cloud_cluster2);
							//se agreega la nube resultante a la ventana para la visualizasion
						   // viewer2->addPointCloud<PointType>(cloud_cluster2, "cloud" + std::to_string(j));
							//se suma uno al contador de cluster
							j++;
							//  std::cout << "cluster" + std::to_string(j) << std::endl;

							if (_debug) {
								std::cout << "tiempo de renderizacion :  " << (double)(clock() - timeStart) / CLOCKS_PER_SEC << std::endl;
								timeStart = clock();
							}

						}
					}
				}
			}


				 //si se usa la tecla de ciclo unico
				if (_ciclo2)
					_ciclo = false;
			
		}
	}

	k2g.shutDown();
	return 0;
}