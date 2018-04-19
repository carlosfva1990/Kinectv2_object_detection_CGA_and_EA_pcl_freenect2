//
// Created by Aksel Sveier. Spring 2016
//

#ifndef RANSACGA_RANSAC_H
#define RANSACGA_RANSAC_H

#endif //RANSACGA_RANSAC_H

#include <space/vsr_cga3D_op.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/passthrough.h>
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>
#include <pcl/filters/extract_indices.h>

#include "objects.h"


using namespace vsr;
using namespace vsr::cga;
using namespace pcl;

//classes that performs ransac operations depending on the object(line, plane, sphere)


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////      SPHERE    ////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This class performs ransac operations with normalized spheres in dual form and pcl point clouds.

class naiveSphere{
public:
	naiveSphere() {
		indexlist = new std::vector<int>;
	}

    float radiusTolerance;
    float inlierTreshold = 0.005; //For the kinect the accuracy is 1 % of the distance from the sensor for 100 images.
    int iterations = 0;       //The inlier treshold is set to a 5mm band around the shape
    int spheresChecked = 0;
    float radius = 0;
    double ratio;
    PointCloud<PointXYZRGB>::Ptr cloud;

    sphere ranSph;
    std::vector<int> *indexlist;

    int numInliers;
    int countedInliers = 0;



	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, float rad, int inliers, double rat) {
		cloud = cl;
		radius = rad;
		radiusTolerance = tol;
		numInliers = inliers;
		ratio = rat;
	}
	void compute() {
		//first create a array of candidate spheres
		vector<int> ran(4);
		int count = 0;
		sphere asphere; //Create a sphere object from object.h
		bool terminate = true;
		iterations = 0;
		while (terminate) {
			iterations++;
			//Random index
			for (int i = 0; i<4; i++) {
				ran[i] = rand() % cloud->points.size();
			}

			//Create dual sphere with indexed points from point cloud, using GA
			asphere.defineDual(cloud, ran); //Creates a dual sphere in confromal space from 4 indexed points in a point cloud

											//Check if sphere is inside tolerance
			if (sphereCheck(asphere)) {
				spheresChecked++;
				//Find number of inlier points
				count = 0;
				for (int j = 0; j < cloud->points.size(); j++) {
					if (isInlier(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z, asphere)) {
						count++;
					}
				}
				if (count > countedInliers) {
					countedInliers = count;
					ranSph = asphere;
				}
			}
			if ((double)countedInliers / (double)numInliers >= ratio) {
				terminate = false;
			}
		}
		ranSph = fit(ranSph);
	}

	sphere fit(sphere sph) {
		delete indexlist;
		indexlist = new std::vector<int>;
		for (int j = 0; j < cloud->points.size(); j++) {
			if (isInlier(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z, sph)) {
				indexlist->push_back(j);
			}
		}

		//Put inlierpoints into matrix for easier handling
		Eigen::MatrixXf P(indexlist->size(), 5);

		for (int i = 0; i < indexlist->size(); i++) {
			P(i, 0) = cloud->points[indexlist->at(i)].x; //x
			P(i, 1) = cloud->points[indexlist->at(i)].y; //y
			P(i, 2) = cloud->points[indexlist->at(i)].z; //x
			P(i, 4) = -0.5*(pow(P(i, 0), 2) + pow(P(i, 1), 2) + pow(P(i, 2), 2));
			P(i, 3) = -1;

		}


		//Compute SVD of matrix
		Eigen::JacobiSVD<Eigen::MatrixXf> USV(P.transpose()*P, Eigen::ComputeFullU | Eigen::ComputeFullV);

		for (int i = 0; i < 5; i++) {
			sph.dualSphere[i] = USV.matrixU()(i, 4) / USV.matrixU()(4, 4);
		}
		sph.dualSphere[3] = USV.matrixU()(4, 4) / USV.matrixU()(4, 4);
		sph.dualSphere[4] = USV.matrixU()(3, 4) / USV.matrixU()(4, 4);
		return sph;
	}


private:
	bool isInlier(float x, float y, float z, sphere can) {
		float dist = sqrt(pow((can.dualSphere[0] - x), 2) + pow((can.dualSphere[1] - y), 2) + pow((can.dualSphere[2] - z), 2));
		if ((dist > (can.radius - inlierTreshold)) && (dist < (can.radius + inlierTreshold))) {
			return true;
		}
		else {
			return false;
		}
	}

	//Check if the radius is inside the tolerance limits
	bool sphereCheck(sphere sph) {
		if (sph.radius < (radius*(1 + radiusTolerance)) && sph.radius >(radius*(1 - radiusTolerance)))
		{
			return true;
		}
		else {
			return false;
		}
	}

};

class naiveSphere2{
public:
	naiveSphere2::naiveSphere2() {
		indexlist = new std::vector<int>;
	}

    float radiusTolerance;
    float inlierTreshold = 0.005; //For the kinect the accuracy is 1 % of the distance from the sensor for 100 images.
    int iterations = 0;       //The inlier treshold is set to a 5mm band around the shape
    int spheresChecked = 0;
    double ratio;
    PointCloud<PointXYZRGB>::Ptr cloud;

    sphere ranSph;
    std::vector<int> *indexlist;

    int numInliers;
    int countedInliers = 0;


	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, int inliers, double rat) {
		cloud = cl;
		radiusTolerance = tol;
		numInliers = inliers;
		ratio = rat;

	}

	void naiveSphere2::compute() {
		//first create a array of candidate spheres
		vector<int> ran(4);
		int count = 0;
		sphere asphere; //Create a sphere object from object.h
		bool terminate = true;
		iterations = 0;
		countedInliers = 0;
		while (terminate) {
			iterations++;
			//Random index
			for (int i = 0; i<4; i++) {
				ran[i] = rand() % cloud->points.size();
			}

			//Create dual sphere with indexed points from point cloud, using GA
			asphere.defineDual(cloud, ran); //Creates a dual sphere in confromal space from 4 indexed points in a point cloud

											//Find number of inlier points
			count = 0;
			for (int j = 0; j < cloud->points.size(); j++) {
				if (isInlier(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z, asphere)) {
					count++;
				}
			}
			if (count > countedInliers) {
				countedInliers = count;
				ranSph = asphere;
			}

			if ((double)countedInliers / (double)numInliers >= ratio) {
				terminate = false;
			}

		}
		ranSph = fit(ranSph);
		spheresChecked = iterations;
	}

	sphere naiveSphere2::fit(sphere sph) {
		delete indexlist;
		indexlist = new std::vector<int>;
		for (int j = 0; j < cloud->points.size(); j++) {
			if (isInlier(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z, sph)) {
				indexlist->push_back(j);
			}
		}

		//Put inlierpoints into matrix for easier handling
		Eigen::MatrixXf P(indexlist->size(), 5);

		for (int i = 0; i < indexlist->size(); i++) {
			P(i, 0) = cloud->points[indexlist->at(i)].x; //x
			P(i, 1) = cloud->points[indexlist->at(i)].y; //y
			P(i, 2) = cloud->points[indexlist->at(i)].z; //x
			P(i, 4) = -0.5*(pow(P(i, 0), 2) + pow(P(i, 1), 2) + pow(P(i, 2), 2));
			P(i, 3) = -1;

		}


		//Compute SVD of matrix
		Eigen::JacobiSVD<Eigen::MatrixXf> USV(P.transpose()*P, Eigen::ComputeFullU | Eigen::ComputeFullV);

		for (int i = 0; i < 5; i++) {
			sph.dualSphere[i] = USV.matrixU()(i, 4) / USV.matrixU()(4, 4);
		}
		sph.dualSphere[3] = USV.matrixU()(4, 4) / USV.matrixU()(4, 4);
		sph.dualSphere[4] = USV.matrixU()(3, 4) / USV.matrixU()(4, 4);
		return sph;
	}


private:

	bool naiveSphere2::isInlier(float x, float y, float z, sphere can) {
		float dist = sqrt(pow((can.dualSphere[0] - x), 2) + pow((can.dualSphere[1] - y), 2) + pow((can.dualSphere[2] - z), 2));
		if ((dist > (can.radius - inlierTreshold)) && (dist < (can.radius + inlierTreshold))) {
			return true;
		}
		else {
			return false;
		}
	}
};


//Class for detecting a sphere in a point cloud with ransac
class ransacSphere{
public:
	//Class constructor
	ransacSphere() {
		indexlist = new std::vector<int>;
	}

    float radiusTolerance;
    float inlierTreshold = 0.005; //For the kinect the accuracy is 1 % of the distance from the sensor for 100 images.
    int iterations = 1000; //Maximum allowed iterations
    int actualIt; //Integer for storing the number of iterations performed
    //float radius = 0; // float for storing the raduis
	float rad;
    PointCloud<PointXYZRGB>::Ptr cloud; //Point cloud for storing the data set

    float time; //Float for storing the computation time of the algorithm
	float maxRadius=0.5;
    Pnt ranSph; //Sphere object
    int candidates = 1; //Number of candidates to detect
    std::vector<int> *indexlist; //Vector for storing inliers
    int numInliers; //Integer for storing the number of inliers


	//Setting the data and parameters of the algoritm
	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol) {
		cloud = cl;
		//radius = rad;
		radiusTolerance = tol;
		candidates = 1;
	}

	//Setting the data and parameters of the algoritm
	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, int cand) {
		cloud = cl;
		//radius = rad;
		radiusTolerance = tol;
		candidates = cand;
	}

	//Runs the primitive shape detection for a sphere
	bool compute() {
		//Variables to keep track of the algorithm
		int it = 0;
		vector<int> ran(4);
		int can = 0;
		vector<sphere> cand(candidates);
		sphere asphere; //Create a sphere object from object.h
		int count;
		vector<int> inPoints(candidates);

		//Timer
		std::clock_t t1, t2;
		t1 = std::clock();

		//Algorithm
		while (it < iterations ){//can < candidates &&  it < iterations) {

			//Generate random indexes
			for (int i = 0; i<4; i++) {
				ran[i] = rand() % cloud->points.size();
			}

			//Create dual sphere with indexed points from point cloud
			asphere.defineDual(cloud, ran); //Creates a dual sphere in confromal space from 4 indexed points in a point cloud

											//Check if sphere is inside tolerance
			if (sphereCheck(asphere)) {

				//Count inliers
				count = 0;
				for (int j = 0; j < cloud->points.size(); j++) {
					if (isInlier(Vec(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z).null(), asphere)) {
						count++;
					}
				}


				///revisar esta parte puede que se tenga que cambiar
				//Check if sphere is feasable
				/*
				if (count >= estimateInliers(asphere)) {
					cand.push_back(asphere);
					inPoints.push_back(count);
					can++;
				}
				*/

				if (inPoints[0] == NULL) {
					inPoints[0] = count;
					cand[0] = asphere;
				}
				if (count > inPoints[0]) {
					cand[0]=asphere;
					inPoints[0]=count;
					can++;
				}
			}
			it++;
		}
		/*
		//Store the performance
		actualIt = it;
		t2 = std::clock();
		time = ((float)t2 - (float)t1) / (CLOCKS_PER_SEC);
		*/

		if (can > 0) {
			//Find candidate with most inliers
			int best = 0;
			numInliers = 0;
			for (int i = 0; i < cand.size(); i++) {
				if (inPoints[i] > numInliers) {
					best = i;
					numInliers = inPoints[i];
				}
			}

			//Fit the sphere to its inliers
			ranSph = fit(cand[best]);
			rad = sqrt((1 / pow(ranSph[3], 2))*(pow(ranSph[0], 2) + pow(ranSph[1], 2) + pow(ranSph[2], 2)) - (2 * ranSph[4] / ranSph[3]));
			//Check if the fit is succesfull, return the unfitted sphere.
			/*if (cand[best].radius / rad > 1.1 || cand[best].radius / rad < 0.9) {
				ranSph = cand[best].dualSphere;
			}*/
			//Return true if a sphere was found
			return true;
		}
		else {
			//Return false if not
			return false;
		}
	}

	//Fitting of a sphere to its inlier point using geometric algebra
	Pnt fit(sphere sph) {
		delete indexlist;
		indexlist = new std::vector<int>;
		for (int j = 0; j < cloud->points.size(); j++) {
			if (isInlier(Vec(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z).null(), sph)) {
				indexlist->push_back(j);
			}
		}

		//Put inlierpoints into matrix for easier handling
		Eigen::MatrixXf P(indexlist->size(), 5);

		for (int i = 0; i < indexlist->size(); i++) {
			P(i, 0) = cloud->points[indexlist->at(i)].x; //x
			P(i, 1) = cloud->points[indexlist->at(i)].y; //y
			P(i, 2) = cloud->points[indexlist->at(i)].z; //x
			P(i, 4) = -0.5*(pow(P(i, 0), 2) + pow(P(i, 1), 2) + pow(P(i, 2), 2));
			P(i, 3) = -1;
		}

		//Compute SVD of matrix
		Eigen::JacobiSVD<Eigen::MatrixXf> USV(P.transpose()*P, Eigen::ComputeFullU | Eigen::ComputeFullV);

		//Normalizing
		for (int i = 0; i < 5; i++) {
			sph.dualSphere[i] = USV.matrixU()(i, 4) / USV.matrixU()(4, 4);
		}
		sph.dualSphere[3] = USV.matrixU()(4, 4) / USV.matrixU()(4, 4);
		sph.dualSphere[4] = USV.matrixU()(3, 4) / USV.matrixU()(4, 4);
		return sph.dualSphere;
	}

private:

	//Function to check if a point is classified as a inlier
	bool isInlier(Pnt point, sphere can) {
		Sca dist = Sca(2)*(point <= can.dualSphere) ;
		if ((dist[0] < inlierTreshold) && (dist[0] > -1* inlierTreshold)) {
			return true;
		}
		else {
			return false;
		}
	}

	//Check if the radius is inside the tolerance limits
	bool sphereCheck(sphere sph) {
		if (sph.radius < maxRadius)
		{
			return true;
		}
		else {
			return false;
		}
	}

	//Function that estimates the inliers, sensor dependent
	int estimateInliers(sphere sph) {
		//a and b are found by experimental measurments with the kinect2 and regression in excel.
		//y = a*(dist^(b)*rad^2)
		float a = 439731.61228317, b = -2.2345982262;
		float rad = sph.radius, dist = sph.dualSphere[2];
		int estimatedInliers = a * (pow(dist, b)*pow(rad, 2));
		if (estimatedInliers > 10) {
			return estimatedInliers;
		}
		else {
			return 999999;
		}
	}

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////// MULTIPLE SPHERES  ////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
class ransacSpheres{
public:
	//Constructor
	ransacSpheres()
		:oct(new pcl::octree::OctreePointCloud<PointXYZRGB>(res)),
		octSearch(new pcl::octree::OctreePointCloudSearch<PointXYZRGB>(res)) {
	}

    PointCloud<PointXYZRGB>::Ptr cloud; //Point cloud for storing the data set
    PointCloud<PointXYZRGB>::Ptr cloud_filtered; //Point cloud for storing the data set after a sphere is extracted
    vector<PointCloud<PointXYZRGB>::Ptr> *subClouds; //Vector for storing the subclouds
    std::vector<pcl::PointIndices::Ptr> *indexlist; //Vector for storing the inlier indices
    std::vector<sphere> *foundSpheres; //Vector for storing the detected spheres

    int totalNumberOfIterations; //Integer for storing the total number of iterations accumulated
    int totalNumberOfPopulatedNodes; //Integer for storing the total number of populated nodes
    int estimatedInliers; //Integer for storing the estimated inliers

    //Default octree bounds
    double x_min = -1, y_min = -1, z_min = 0, x_max = 1, y_max = 1, z_max = 2;
    double rootLength = 0;
    float res = 0.02;
    int depth;




	//Setting the data and parameters of the algoritm
	void setData(PointCloud<PointXYZRGB>::Ptr cl, float resolution) {
		cloud = cl;
		cloud_filtered = cl;
		res = resolution;
		divideCloud(); //Divides the cloud to 4 random subsets, used for faster scoring
		indexlist = new std::vector<pcl::PointIndices::Ptr>;
		foundSpheres = new std::vector<sphere>;
	}

	//Function that sets the bounds of the octree root node
	void setOctreeBounds(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax) {
		x_min = xmin; y_min = ymin; z_min = zmin; x_max = xmax; y_max = ymax; z_max = zmax;
		oct->deleteTree();
		oct->setResolution(res);
		oct->defineBoundingBox(x_min, y_min, z_min, x_max, y_max, z_max);
		double xma, yma, zma, xm, ym, zm;
		oct->getBoundingBox(xm, ym, zm, xma, yma, zma);
		rootLength = sqrt(pow(xma - xm, 2));

		//Start at rootlength
		oct->deleteTree();
		oct->setResolution(rootLength);
		oct->defineBoundingBox(x_min, y_min, z_min, x_max, y_max, z_max);
		octSearch->deleteTree();
		octSearch->setResolution(rootLength);
		octSearch->defineBoundingBox(x_min, y_min, z_min, x_max, y_max, z_max);
	}

	//Function that creates octree for point picking
	void setOctree(float resolution) {
		oct->deleteTree();
		oct->setResolution(resolution);
		oct->defineBoundingBox(x_min, y_min, z_min, x_max, y_max, z_max);
		double xma, yma, zma, xm, ym, zm;
		oct->getBoundingBox(xm, ym, zm, xma, yma, zma);
		rootLength = sqrt(pow(xma - xm, 2));
		oct->setInputCloud(cloud_filtered);
		oct->addPointsFromInputCloud();
	}

	//Function that creates octree for searching
	void setSearchOctree(float resolution) {
		octSearch->deleteTree();
		octSearch->setResolution(resolution);
		octSearch->defineBoundingBox(x_min, y_min, z_min, x_max, y_max, z_max);
		octSearch->setInputCloud(subClouds->at(0));
		octSearch->addPointsFromInputCloud();
	}

	//Runs the multipe shape detection algorithm
	void compute() {
		//Variables to keep track of the algorithm
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
		std::vector<int> indices;
		pcl::ExtractIndices<pcl::PointXYZRGB> extract;
		octree::OctreePointCloud<PointXYZRGB>::LeafNodeIterator it;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
		bool restart = false;
		float thisRes = rootLength;
		totalNumberOfIterations = 0;
		totalNumberOfPopulatedNodes = 0;

		//Iterates through the octree
		while (thisRes >= res) {

			setOctree(thisRes);
			setSearchOctree(thisRes);
			int tempInl = 0;
			int bestInl = 0;

			//Iterate throug all nodes at present level
			it.reset();
			it = oct->leaf_begin();
			sphere asphere;
			sphere tempSphere;
			while (*++it && !restart) {
				bestInl = 0;
				tempInl = 0;
				//Only consider populated nodes
				if (it.isLeafNode()) {
					indices.resize(0);
					it.getLeafContainer().getPointIndices(indices);
					//Only consider nodes with more than 4 points
					if (indices.size() > 4) {
						totalNumberOfPopulatedNodes++;
						//Initialzes 4 spheres in the current node
						for (int i = 0; i < 4; i++) {
							totalNumberOfIterations++;
							indices = shuffle(indices);
							tempSphere.defineDual(cloud_filtered, indices);
							//Store the sphere with the most inliers
							if (tempSphere.z < 3 && tempSphere.z > 0.2 && tempSphere.radius > 0.015 && tempSphere.radius < 0.13) {
								tempInl = getInliers(tempSphere);
								if (bestInl < tempInl) {
									bestInl = tempInl;
									asphere = tempSphere;
									tempInl = 0;
								}
							}

						}
						//Score the sphere, the estimate have to be divided by 4 because only a quater of the total point is used for scoring
						if ((bestInl >= estimateInliers(asphere)*0.25)) {
							//Find and store inliers all the inliers
							inliers.reset(new pcl::PointIndices());
							for (int i = 0; i < cloud_filtered->points.size(); i++) {
								if (isInlier(cloud_filtered->points[i].x, cloud_filtered->points[i].y, cloud_filtered->points[i].z, asphere)) {
									inliers->indices.push_back(i);
								}
							}

							//All of the inliers are checked against the estimate
							if (inliers->indices.size() >= estimateInliers(asphere)) {
								//Sphere is accepted
								indexlist->push_back(inliers);

								//Fit the sphere
								sphere fitted = fit(asphere, inliers);
								if (fitted.radius / asphere.radius > 1.1 || fitted.radius / asphere.radius < 0.9) {
									std::cout << "Bad fitting!" << std::endl;
									foundSpheres->push_back(asphere);
								}
								else {
									foundSpheres->push_back(fitted);
								}

								//Extract inliers from the point cloud
								extract.setInputCloud(cloud_filtered);
								extract.setIndices(inliers);
								extract.setNegative(true);
								extract.filter(*cloud_f);
								cloud_filtered.reset();
								cloud_filtered = cloud_f;

								//Reset the octrees and restart the iteration at the same resolution
								restart = true;
							}

						}
					}
				}
			}
			if (!restart) {
				//No sphere was found at the current level
				//Continue iteration at the lower level
				thisRes = thisRes / 2;
			}
			else {
				//A sphere was found and the iteration is restarted at the current level
				restart = false;
				divideCloud();
			}
		}
	}

	//Function that fits a sphere to its inlier point using conformal geometric algebra
	sphere fit(sphere sph, pcl::PointIndices::Ptr inliers) {

		//Put inlierpoints into matrix for easier handling
		Eigen::MatrixXf P(inliers->indices.size(), 5);
		for (int i = 0; i < inliers->indices.size(); i++) {
			P(i, 0) = cloud_filtered->points[inliers->indices.at(i)].x; //x
			P(i, 1) = cloud_filtered->points[inliers->indices.at(i)].y; //y
			P(i, 2) = cloud_filtered->points[inliers->indices.at(i)].z; //x
			P(i, 4) = -0.5*(pow(P(i, 0), 2) + pow(P(i, 1), 2) + pow(P(i, 2), 2));
			P(i, 3) = -1;
		}

		//Compute SVD of matrix
		Eigen::JacobiSVD<Eigen::MatrixXf> USV(P.transpose()*P, Eigen::ComputeFullU | Eigen::ComputeFullV);

		//Normalize
		Pnt temp;
		for (int i = 0; i < 5; i++) {
			temp[i] = USV.matrixU()(i, 4) / USV.matrixU()(4, 4);
		}
		temp[3] = USV.matrixU()(4, 4) / USV.matrixU()(4, 4);
		temp[4] = USV.matrixU()(3, 4) / USV.matrixU()(4, 4);
		float radius = sqrt((1 / pow(temp[3], 2))*(pow(temp[0], 2) + pow(temp[1], 2) + pow(temp[2], 2)) - (2 * temp[4] / temp[3]));
		sph.defineDual(temp[0], temp[1], temp[2], radius);
		return sph;
	}

	//Function that divides the point cloud to 4 random subsets, used for faster scoring
	void divideCloud() {
		subClouds = new vector<PointCloud<PointXYZRGB>::Ptr>;
		PointCloud<PointXYZRGB>::Ptr temp(new PointCloud<PointXYZRGB>);
		vector<int> list(cloud_filtered->points.size());
		//Store all the indexes in a vector
		for (int i = 0; i < cloud_filtered->points.size(); i++) {
			list[i] = i;
		}
		//Shuffle the vector
		list = shuffle(list);

		//Split the point cloud into 4 "subclouds"
		int count = 1;
		for (int i = 0; i < cloud_filtered->points.size(); i++) {
			temp->push_back(cloud_filtered->points[list[i]]);
			if (i == ((cloud_filtered->points.size() / 4)*count - 1)) {
				subClouds->push_back(temp);
				temp.reset(new PointCloud<PointXYZRGB>);
				count++;
			}
		}

		//Put the "rest" in the first subcloud
		for (int i = cloud_filtered->points.size() - 1; i > cloud_filtered->points.size() - 1 - (cloud_filtered->points.size() % 4); i--) {
			subClouds->at(0)->push_back(cloud_filtered->points[i]);
		}
	}


private:
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr octSearch; //Search octree object
    pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::Ptr oct; //Point picking octree object


	//Function for suffeling a list of integers
	vector<int> shuffle(vector<int> list) {
		int ran, temp;
		for (int i = list.size(); i > 0; i--) {
			ran = rand() % i;
			temp = list[i - 1];
			list[i - 1] = list[ran];
			list[ran] = temp;
		}
		return list;
	}

	//Function that counts and return the inliers of a sphere for a subcloud
	int getInliers(sphere can) {
		int inl = 0;
	pcl:PointXYZRGB searchPoint;
		std::vector<int> pointIdxVec;
		std::vector<float> pointNKNSquaredDistance;

		searchPoint.x = can.x;
		searchPoint.y = can.y;
		searchPoint.z = can.z;
		octSearch->radiusSearch(searchPoint, can.radius + 0.05, pointIdxVec, pointNKNSquaredDistance);
		float inlierTreshold = (7.0f / 1800.0f) + (1.0f / 18.0f)*can.radius; //Interpolation function
		for (int i = 0; i < pointIdxVec.size(); i++) {
			if ((sqrt(pointNKNSquaredDistance[i]) < (can.radius + inlierTreshold)) && (sqrt(pointNKNSquaredDistance[i]) > (can.radius - inlierTreshold))) {
				inl++;
			}
		}
		return inl;
	}

	//Function to check if a point is classified as a inlier
	bool isInlier(float x, float y, float z, sphere can) {
		float dist = sqrt(pow((can.dualSphere[0] - x), 2) + pow((can.dualSphere[1] - y), 2) + pow((can.dualSphere[2] - z), 2));
		float inlierTreshold = (7.0f / 1800.0f) + (1.0f / 18.0f)*can.radius; //Interpolation function
		if ((dist > (can.radius - inlierTreshold)) && (dist < (can.radius + inlierTreshold))) {
			return true;
		}
		else {
			return false;
		}
	}

	//Function that estimates the number of inliers, used to decide whether a sphere is accepted
	int estimateInliers(sphere sph) {
		//a and b are found by experimental measurments with the kinect2 and regression in excel.
		//y = a*(dist^(b)*rad^2)
		float a = 439731.61228317, b = -2.2345982262;
		float rad = sph.radius, dist = sph.dualSphere[2];
		estimatedInliers = a * (pow(dist, b)*pow(rad, 2));
		if (estimatedInliers > 10) {
			return estimatedInliers;
		}
		else {
			return 999999;
		}
	}
};
*/
//////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////      PLANE     ////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Class for detecting a plane in a point cloud with RANSAC
class ransacPlane{
public:
	//Constructor
	ransacPlane() {
		indexlist = new std::vector<int>;
	}
    float planetol; //Tolerance used for deciding wether a point is compatable with a plane
    PointCloud<PointXYZRGB>::Ptr cloud; //Point cloud for holding the data set
    PointCloud<PointXYZRGB>::Ptr segment; //Point cloud for holding the inlier points
    std::vector<int> *indexlist; //Vector for holding the index of inlier points

	int iterations = 1000; //Maximum allowed iterations
    Pnt ranPln; //Plane object represented in IPNS 
    int candidates; //Number of candidates to detect

	//Setting the data and parameters of the algoritm
	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, int cand) {
		cloud = cl;
		planetol = tol;
		candidates = cand;
	}

	//Runs the primitive shape detection for a plane
	bool compute() {
		//Variables to keep track of the algorithm
		int ran[3];
		int count;
		vector<Pnt> cand(candidates);
		plane aplane; //Create a plane object from object.h
		int can = 0;
		int it = 0;
		vector<int> inPoints(candidates);

					  //Algorithm
		while (it < iterations) {

			//Generate random indexesx
			for (int i = 0; i<3; i++) {
				ran[i] = rand() % cloud->points.size();
			}
			//Create dual plane with indexed points from point cloud, using GA
			aplane.defineDual(cloud, ran); //Creates a dual plane in confromal space from 3 indexed points in a point cloud
			cand[0] = aplane.normDualPlane;

			//Find number of inlier points for each candidate
			count = 0;
			for (int j = 0; j < cloud->points.size(); j++) {
				if (isInlier(Vec(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z).null(), cand[0])) {
					count++;
				}
			}


			if (inPoints[0] == NULL) {
				inPoints[0] = count;
			}

			if (count > inPoints[0]) {
				cand[0] = aplane.normDualPlane;
				inPoints[0] = count;
				can++;
			}
			
			it++;
		}

		if (can > 0) {
			/*
			//Find candidate with most inliers
			int best = 0;
			int numBest = 0;
			for (int i = 0; i < cand.size(); i++) {
				if (inPoints[i] > numBest) {
					best = i;
					numBest = inPoints[i];
				}
			}
			*/
			//Fit the plane with the most inilers to its inliers
			//Pnt fittedPln = fit(cand[0]);

			/*
			//Finally normalize
			for (int i = 0; i < 5; i++) {
				ranPln[i] = fittedPln[i] / sqrt(pow(fittedPln[0], 2) + pow(fittedPln[1], 2) + pow(fittedPln[2], 2));
			}
			*/

			delete indexlist;
			indexlist = new std::vector<int>;
			for (int j = 0; j < cloud->points.size(); j++) {
				if (isInlier(Vec(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z).null(), cand[0])) {
					indexlist->push_back(j);
				}
			}

			ranPln = cand[0];
			return true;
		}
		else {
			return false;
		}

	}

	//Function that creates and stores the indexlist of the current plane
	void segmentPlane(PointCloud<PointXYZRGB>::Ptr seg) {
		delete indexlist;
		indexlist = new std::vector<int>;
		for (int j = 0; j < seg->points.size(); j++)
			if ( isInlier(  Vec( seg->points[j].x, seg->points[j].y, seg->points[j].z ).null(), ranPln ) ) {
				indexlist->push_back(j);
			}
	}

	//Fitting of a plane to its inlier point using geometric algebra
	Pnt fit(Pnt pln) {
	
		delete indexlist;
		indexlist = new std::vector<int>;
		for (int j = 0; j < cloud->points.size(); j++) {
			if (isInlier(Vec(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z).null(), pln)) {
				indexlist->push_back(j);
			}
		}

		
		//Put inlierpoints into matrix for easier handling
		Eigen::MatrixXf P(indexlist->size(), 5);

		for (int i = 0; i < indexlist->size(); i++) {
			P(i, 0) = cloud->points[indexlist->at(i)].x; //x
			P(i, 1) = cloud->points[indexlist->at(i)].y; //y
			P(i, 2) = cloud->points[indexlist->at(i)].z; //z
			P(i, 4) = -0.5*(pow(P(i, 0), 2) + pow(P(i, 1), 2) + pow(P(i, 2), 2));
			P(i, 3) = -1;
		}

		//Compute SVD of matrix
		Eigen::JacobiSVD<Eigen::MatrixXf> USV(P.transpose()*P, Eigen::ComputeFullU | Eigen::ComputeFullV);
		
		for (int i = 0; i < 3; i++) {
			pln[i] = USV.matrixU()(i, 4);//USV.matrixU()(3,0);
		}
		pln[3] = USV.matrixU()(4, 4);//USV.matrixU()(4,0);
		pln[4] = USV.matrixU()(3, 4);//USV.matrixU()(4,0);
		
		return pln;
	}

private:

	//Function to check if a point is classified as a inlier
	bool isInlier(Pnt point, Pnt plane) {
		Sca distance = point <= plane;
		if (distance[0] < planetol) {
			return true;
		}
		else {
			return false;
		}
	}

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////      CIRCLE     ////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

class ransacCircle{
public:
    PointCloud<PointXYZRGB>::Ptr cloud;
    float circletol = 0;
    float searchRadius = 0;
    int candidates = 3000;
    circle ranCir;

	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, float rad) {
		cloud = cl;
		circletol = tol;
		searchRadius = rad;
	}

	void compute() {
		//first create a array of candidate spheres
		srand(time(NULL));
		int ran[3];
		int ccount = 0;
		int icount;
		int best = 0;
		circle acircle; //Create a circle object from object.h
		while (ccount < candidates) {
			//Random index
			for (int i = 0; i<3; i++) {
				ran[i] = rand() % cloud->points.size();
			}
			//Create dual sphere with indexed points from point cloud, using GA
			acircle.defineCircle(cloud, ran); //Creates a circle in confromal space from 4 indexed points in a point cloud

											  //Check if sphere is inside tolerance
			if (radiusCheck(acircle.radius)) {
				//Find number of inliers for sphere
				icount = 0;
				for (int i = 0; i < cloud->points.size(); i++) {
					if (isInlier(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, acircle.circleCenter, acircle.plane)) {
						icount++;
					}
				}
				cout << icount << endl;
				if (icount > best) {
					best = icount;
					ranCir.defineCircle(cloud, ran);
				}
				cout << "candidate found" << endl;
				ccount++;
			}
		}
	}

private:
	bool isInlier(float x, float y, float z, Pnt center, Pnt plane) {
		float dist = sqrt(pow(x - center[0], 2) + pow(y - center[1], 2) + pow(z - center[2], 2));
		if (dist < (searchRadius - (searchRadius*circletol)) || dist >(searchRadius + (searchRadius*circletol))) {
			return false;
		}
		else if ((0.5 * (Vec(x, y, z).null() <= plane)[0]) > 0.005) {
			return false;
		}
		else {
			return true;
		}
	}

	bool radiusCheck(float rad) {
		if (rad > (searchRadius - (searchRadius*circletol)) && rad < (searchRadius + (searchRadius*circletol))) {
			return true;
		}
		else {
			return false;
		}
	}

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////      CYLINDER     ////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Class for detecting a cylinder in a point cloud  using the plane-circle approach.
class ransacCylinder{
public:
    PointCloud<PointXYZRGB>::Ptr cloud; //Point cloud for storing the data set
    PointCloud<PointXYZRGB>::Ptr inlierCloud; //Point cloud for storing the inliers
    float tol = 0; //Float for the radius tolerance
    float searchRadius = 0; //Float for storing the radius
    int numberInliers; //Float for storing the number of inliers
    cylinder ranCyl; //cylinder object
    int iterations = 1; //Maximum number of iterations
    int candidates; //Number of candidates to detect

    //Length
    Eigen::Vector4f centroid; //Centroid of inliers
    Eigen::Vector3f projL; //Point at the edge of cylinder on the cylinder axis
    Eigen::Vector3f projS; //Point at the edge of cylinder on the cylinder axis
    int largestInd = 0; //Index of point at cylinder edge
    int smallestInd = 0; //Index of point at cylinder edge


	//Setting the data and parameters of the algoritm
	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tolerance, float rad, int inliers, int cand) {
		cloud = cl;
		tol = tolerance;
		searchRadius = rad;
		numberInliers = inliers;
		candidates = cand;
	}

	//Runs the primitive shape detection for a cylinder using the plane-circle approach
	void compute() {
		//Variables to keep track of the algorithm
		int ran[3];
		int icount = 0;
		int can = 0;
		vector<cylinder> cand;
		vector<int> inPoints;
		bool terminate = true;
		cylinder acylinder; //Create a cylinder object from object.h
		int it = 0;

		//Algorithm
		while (can < candidates && it < iterations) {
			it++;
			//Generate random indexes
			for (int i = 0; i<3; i++) {
				ran[i] = rand() % cloud->points.size();
			}
			//Create cylinder with indexed points from point cloud, using GA
			acylinder.defineCylinder(cloud, ran);

			//Check if cylinder is inside radius tolerance
			if (radiusCheck(acylinder.radius)) {

				//Find number of inliers for cylinder
				Pnt p1 = acylinder.center;
				Pnt p2;
				p2[0] = p1[0] + acylinder.plan[0];
				p2[1] = p1[1] + acylinder.plan[1];
				p2[2] = p1[2] + acylinder.plan[2];
				icount = 0;
				for (int i = 0; i < cloud->points.size(); i++) {
					if (isInlier(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, p1, p2)) {
						icount++;
					}
				}
				//Store cylinder and number of inliers
				cand.push_back(acylinder);
				inPoints.push_back(icount);
			}
		}

		if (can > 0) {
			//Find candidate with most inliers
			int best = 0;
			int numInliers = 0;
			for (int i = 0; i < cand.size(); i++) {
				if (inPoints[i] > numInliers) {
					best = i;
					numInliers = inPoints[i];
				}
			}
			//Store the cylinder with the most inliers
			ranCyl = cand[best];
		}
	}

	//Function that finds the cylinder length from the inlierpoints
	void cylinderLength() {
		inlierCloud.reset(new PointCloud<PointXYZRGB>);
		//First find all inliers and store in a point cloud
		cylinder acylinder = ranCyl;
		Pnt p1 = acylinder.center;
		Pnt p2;
		p2[0] = p1[0] + acylinder.plan[0];
		p2[1] = p1[1] + acylinder.plan[1];
		p2[2] = p1[2] + acylinder.plan[2];
		for (int i = 0; i < cloud->points.size(); i++) {
			if (isInlier(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, p1, p2)) {
				inlierCloud->push_back(cloud->points[i]);
			}
		}
		//Normalize
		pcl::compute3DCentroid(*inlierCloud, centroid);

		for (int i = 0; i < inlierCloud->size(); i++) {
			inlierCloud->points[i].x -= centroid[0];
			inlierCloud->points[i].y -= centroid[1];
			inlierCloud->points[i].z -= centroid[2];
		}

		//Find point farthest away from origo
		float largestD = 0;
		float thisD = 0;
		for (int i = 0; i < inlierCloud->size(); i++) {
			thisD = sqrt(pow(inlierCloud->points[i].x, 2) + pow(inlierCloud->points[i].y, 2) + pow(inlierCloud->points[i].z, 2));
			if (largestD < thisD) {
				largestD = thisD;
				largestInd = i;
			}
		}

		//Find the farthest point in the opposite direction.
		//This is done by only searching in the opposite octant of the first point
		float smallestD = 0;
		thisD = 0;
		for (int i = 0; i < inlierCloud->size(); i++) {
			if ((inlierCloud->points[i].x * inlierCloud->points[largestInd].x) < 0 && (inlierCloud->points[i].y * inlierCloud->points[largestInd].y) < 0 && (inlierCloud->points[i].z * inlierCloud->points[largestInd].z) < 0) {
				thisD = sqrt(pow(inlierCloud->points[i].x, 2) + pow(inlierCloud->points[i].y, 2) + pow(inlierCloud->points[i].z, 2));
				if (smallestD < thisD) {
					smallestD = thisD;
					smallestInd = i;
				}
			}
		}

		//Translate back to original position
		for (int i = 0; i < inlierCloud->size(); i++) {
			inlierCloud->points[i].x += centroid[0];
			inlierCloud->points[i].y += centroid[1];
			inlierCloud->points[i].z += centroid[2];
		}

		//Find lines from "center" to largest and smallest
		Eigen::Vector3f cl(inlierCloud->points[largestInd].x - ranCyl.center[0], inlierCloud->points[largestInd].y - ranCyl.center[1], inlierCloud->points[largestInd].z - ranCyl.center[2]);
		Eigen::Vector3f cs(inlierCloud->points[smallestInd].x - ranCyl.center[0], inlierCloud->points[smallestInd].y - ranCyl.center[1], inlierCloud->points[smallestInd].z - ranCyl.center[2]);


		//Project the lines onto the centerline
		Eigen::Vector3f centerline(ranCyl.plan[0], ranCyl.plan[1], ranCyl.plan[2]);

		projL = cl.dot(centerline) * centerline;
		projS = cs.dot(centerline) * centerline;
	}

private:
	//Check if the radius is inside the tolerance limits
	bool radiusCheck(float rad) {
		if (rad > (searchRadius - (searchRadius*tol)) && rad < (searchRadius + (searchRadius*tol))) {
			return true;
		}
		else {
			return false;
		}
	}

	//Function to check if a point is classified as a inlier
	bool isInlier(float x, float y, float z, Pnt p1, Pnt p2) {
		Eigen::Vector3d v(x - p1[0], y - p1[1], z - p1[2]);
		Eigen::Vector3d w(x - p2[0], y - p2[1], z - p2[2]);
		Eigen::Vector3d q(p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]);
		Eigen::Vector3d cross = v.cross(w);
		float d = sqrt(pow(cross[0], 2) + pow(cross[1], 2) + pow(cross[2], 2)) / sqrt(pow(q[0], 2) + pow(q[1], 2) + pow(q[2], 2));
		if (d < (searchRadius*(1 + tol)) && d >(searchRadius*(1 - tol))) {
			return true;
		}
		else {
			return false;
		}
	}

};

//Class for detecting a cylinder in a point cloud using the sphere-sphere approach.
class ransacCylinder2{
public:
	//Constructor
	ransacCylinder2() {
		inlierCloud.reset(new PointCloud<PointXYZRGB>);
	}
    PointCloud<PointXYZRGB>::Ptr cloud; //Point cloud for storing the data set
    PointCloud<PointXYZRGB>::Ptr inlierCloud; //Point cloud for storing the inliers
    float tol = 0; //Float for the radius tolerance
    float searchRadius = 0; //Float for storing the radius
    int inliers; //Integer for storing the number of inliers
    cylinder ranCyl; //Cylinder object
    ransacSphere ball; //Sphere detection object

    //Length
    Eigen::Vector4f centroid; //Centroid of inliers
    Eigen::Vector3f projL; //Point at the edge of cylinder on the cylinder axis
    Eigen::Vector3f projS; //Point at the edge of cylinder on the cylinder axis
    int largestInd = 0; //Index of point at cylinder edge
    int smallestInd = 0; //Index of point at cylinder edge

						 //Setting the data and parameters of the algoritm
	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tolerance, float rad) {
		cloud = cl;
		tol = tolerance;
		searchRadius = rad;
	}

	//Runs the primitive shape detection for a cylinder using the sphere-sphere approach
	void compute() {
		//Variables to keep track of the algorithm
		inlierCloud.reset(new PointCloud<PointXYZRGB>);
		int icount;
		Pnt p1, p2;
		ball.setData(cloud, tol, searchRadius); //Ransac for spheres, data is set

												//Find the first sphere that satisfies the radius
		ball.compute();
		p1 = ball.ranSph;

		//Find the second sphere that satisfies the radius
		ball.compute();
		p2 = ball.ranSph;

		//Count the inliers for the cylinder and store the inliers in a inlier cloud
		for (int i = 0; i < cloud->points.size(); i++) {
			if (isInlier2(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, p1, p2)) {
				inlierCloud->push_back(cloud->points[i]);
				icount++;
			}
		}
		inliers = icount;

		//The cylinder is created and stored.
		ranCyl.defineCylinder(p1, p2, searchRadius);
	}

	//Function that finds the cylinder length from the inlierpoints
	void cylinderLength(cylinder cyl) {

		//Normalize
		pcl::compute3DCentroid(*inlierCloud, centroid);

		for (int i = 0; i < inlierCloud->size(); i++) {
			inlierCloud->points[i].x -= centroid[0];
			inlierCloud->points[i].y -= centroid[1];
			inlierCloud->points[i].z -= centroid[2];
		}

		//Find point farthest away from origo
		float largestD = 0;
		float thisD = 0;
		for (int i = 0; i < inlierCloud->size(); i++) {
			thisD = sqrt(pow(inlierCloud->points[i].x, 2) + pow(inlierCloud->points[i].y, 2) + pow(inlierCloud->points[i].z, 2));
			if (largestD < thisD) {
				largestD = thisD;
				largestInd = i;
			}
		}

		//Find the farthest point in the opposite direction.
		//This is done by only searching in the opposite octant of the first point
		float smallestD = 0;
		thisD = 0;
		for (int i = 0; i < inlierCloud->size(); i++) {
			if ((inlierCloud->points[i].x * inlierCloud->points[largestInd].x) < 0 && (inlierCloud->points[i].y * inlierCloud->points[largestInd].y) < 0 && (inlierCloud->points[i].z * inlierCloud->points[largestInd].z) < 0) {
				thisD = sqrt(pow(inlierCloud->points[i].x, 2) + pow(inlierCloud->points[i].y, 2) + pow(inlierCloud->points[i].z, 2));
				if (smallestD < thisD) {
					smallestD = thisD;
					smallestInd = i;
				}
			}
		}

		//Translate back to original position
		for (int i = 0; i < inlierCloud->size(); i++) {
			inlierCloud->points[i].x += centroid[0];
			inlierCloud->points[i].y += centroid[1];
			inlierCloud->points[i].z += centroid[2];
		}

		//Find lines from "center" to largest and smallest
		Eigen::Vector3f cl(inlierCloud->points[largestInd].x - ranCyl.center[0], inlierCloud->points[largestInd].y - ranCyl.center[1], inlierCloud->points[largestInd].z - ranCyl.center[2]);
		Eigen::Vector3f cs(inlierCloud->points[smallestInd].x - ranCyl.center[0], inlierCloud->points[smallestInd].y - ranCyl.center[1], inlierCloud->points[smallestInd].z - ranCyl.center[2]);


		//Project the lines onto the centerline
		Eigen::Vector3f centerline(ranCyl.plan[0], ranCyl.plan[1], ranCyl.plan[2]);

		projL = cl.dot(centerline) * centerline;
		projS = cs.dot(centerline) * centerline;
	}

private:
	//Function to check if a point is classified as a inlier
	bool isInlier2(float x, float y, float z, Pnt p1, Pnt p2) {
		Eigen::Vector3d v(x - p1[0], y - p1[1], z - p1[2]);
		Eigen::Vector3d w(x - p2[0], y - p2[1], z - p2[2]);
		Eigen::Vector3d q(p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]);
		Eigen::Vector3d cross = v.cross(w);
		float d = sqrt(pow(cross[0], 2) + pow(cross[1], 2) + pow(cross[2], 2)) / sqrt(pow(q[0], 2) + pow(q[1], 2) + pow(q[2], 2));
		if (d < (searchRadius*(1 + tol)) && d >(searchRadius*(1 - tol))) {
			return true;
		}
		else {
			return false;
		}
	}

};

//Cylinder class used for experiments
class ransacCylinder22{
public:
    PointCloud<PointXYZRGB>::Ptr cloud;
    PointCloud<PointXYZRGB>::Ptr inlierCloud;
    float tol = 0;
    float searchRadius = 0;
    int numberInliers;
    double ratio;
    cylinder ranCyl;

    int iterations = 0;
    int spheresIterations = 0;
    int cylindersChecked = 0;
    int countedInliers;

	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tolerance, float rad, int inliers, double rat) {
		cloud = cl;
		tol = tolerance;
		searchRadius = rad;
		numberInliers = inliers;
		ratio = rat;
	}

	void compute() {
		int ran[3];
		int icount = 0;
		bool terminate = true;
		Pnt p1, p2;
		int itB1, itB2;
		iterations = 0;

		while (terminate) {
			naiveSphere ball1;
			ball1.setData(cloud, tol, searchRadius, 6000, ratio);
			naiveSphere ball2;
			ball2.setData(cloud, tol, searchRadius, 6000, ratio);
			iterations++;
			icount = 0;
			ball1.compute();
			p1 = ball1.ranSph.dualSphere;
			itB1 = ball1.iterations;
			ball2.compute();
			itB2 = ball2.iterations;
			p2 = ball2.ranSph.dualSphere;
			spheresIterations = spheresIterations + itB1 + itB2;

			for (int i = 0; i < cloud->points.size(); i++) {
				if (isInlier(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, p1, p2)) {
					icount++;
				}
			}
			if ((double)icount / (double)numberInliers >= ratio) {

				countedInliers = icount;
				ranCyl.defineCylinder(p1, p2, searchRadius);
				terminate = false;
			}

		}
	}


    //Lengrth
    Eigen::Vector4f centroid;

    Eigen::Vector3f projL;
    Eigen::Vector3f projS;

    int largestInd = 0;
    int smallestInd = 0;

	void cylinderLength() {
		inlierCloud.reset(new PointCloud<PointXYZRGB>);
		//First find all inliers and store in a point cloud
		//Find number of inliers for cylinder
		cylinder acylinder = ranCyl;
		Pnt p1 = acylinder.center;
		Pnt p2;
		p2[0] = p1[0] + acylinder.plan[0];
		p2[1] = p1[1] + acylinder.plan[1];
		p2[2] = p1[2] + acylinder.plan[2];
		for (int i = 0; i < cloud->points.size(); i++) {
			if (isInlier(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, p1, p2)) {
				inlierCloud->push_back(cloud->points[i]);
			}
		}
		//Normalize
		pcl::compute3DCentroid(*inlierCloud, centroid);

		for (int i = 0; i < inlierCloud->size(); i++) {
			inlierCloud->points[i].x -= centroid[0];
			inlierCloud->points[i].y -= centroid[1];
			inlierCloud->points[i].z -= centroid[2];
		}

		//Find point farthest away from origo
		float largestD = 0;
		float thisD = 0;
		for (int i = 0; i < inlierCloud->size(); i++) {
			thisD = sqrt(pow(inlierCloud->points[i].x, 2) + pow(inlierCloud->points[i].y, 2) + pow(inlierCloud->points[i].z, 2));
			if (largestD < thisD) {
				largestD = thisD;
				largestInd = i;
			}
		}

		//Find the farthest point in the opposite direction.
		//This is done by only searching in the opposite octant of the first point
		float smallestD = 0;
		thisD = 0;
		for (int i = 0; i < inlierCloud->size(); i++) {
			if ((inlierCloud->points[i].x * inlierCloud->points[largestInd].x) < 0 && (inlierCloud->points[i].y * inlierCloud->points[largestInd].y) < 0 && (inlierCloud->points[i].z * inlierCloud->points[largestInd].z) < 0) {
				thisD = sqrt(pow(inlierCloud->points[i].x, 2) + pow(inlierCloud->points[i].y, 2) + pow(inlierCloud->points[i].z, 2));
				if (smallestD < thisD) {
					smallestD = thisD;
					smallestInd = i;
				}
			}
		}

		//Translate back to original position
		for (int i = 0; i < inlierCloud->size(); i++) {
			inlierCloud->points[i].x += centroid[0];
			inlierCloud->points[i].y += centroid[1];
			inlierCloud->points[i].z += centroid[2];
		}

		//Find lines from "center" to largest and smallest
		Eigen::Vector3f cl(inlierCloud->points[largestInd].x - ranCyl.center[0], inlierCloud->points[largestInd].y - ranCyl.center[1], inlierCloud->points[largestInd].z - ranCyl.center[2]);
		Eigen::Vector3f cs(inlierCloud->points[smallestInd].x - ranCyl.center[0], inlierCloud->points[smallestInd].y - ranCyl.center[1], inlierCloud->points[smallestInd].z - ranCyl.center[2]);


		//Project the lines onto the centerline
		Eigen::Vector3f centerline(ranCyl.plan[0], ranCyl.plan[1], ranCyl.plan[2]);

		projL = cl.dot(centerline) * centerline;
		projS = cs.dot(centerline) * centerline;
	}

private:
	bool radiusCheck(float rad) {
		if (rad > (searchRadius - (searchRadius*tol)) && rad < (searchRadius + (searchRadius*tol))) {
			return true;
		}
		else {
			return false;
		}
	}

	bool isInlier(float x, float y, float z, Pnt p1, Pnt p2) {

		Eigen::Vector3d v(x - p1[0], y - p1[1], z - p1[2]);
		Eigen::Vector3d w(x - p2[0], y - p2[1], z - p2[2]);
		Eigen::Vector3d q(p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2]);
		Eigen::Vector3d cross = v.cross(w);
		float d = sqrt(pow(cross[0], 2) + pow(cross[1], 2) + pow(cross[2], 2)) / sqrt(pow(q[0], 2) + pow(q[1], 2) + pow(q[2], 2));
		if (d < (searchRadius*(1 + tol)) && d >(searchRadius*(1 - tol))) {
			return true;
		}
		else {
			return false;
		}
	}

};

