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

#include <cmath>        // std::abs
#include "objects.h"


using namespace vsr;
using namespace vsr::cga;
using namespace pcl;

//classes that performs ransac operations depending on the object(line, plane, sphere)


//////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////      SPHERE    ////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Class for detecting a sphere in a point cloud with ransac
class ransacSphere{
public:
	//Class constructor
	ransacSphere() {
		indexlist = new std::vector<int>;
	}

	float rad=0;
    float radiusTolerance=0.001;
    float inlierTreshold = 0.005; //For the kinect the accuracy is 1 % of the distance from the sensor for 100 images.
	float maxRadius = 1;
	int candidates = 1; //Number of candidates to detect
	int iterations = 1000; //Maximum allowed iterations
    int actualIt; //Integer for storing the number of iterations performed
	int numInliers; //Integer for storing the number of inliers
	std::vector<int> *indexlist; //Vector for storing inliers
    PointCloud<PointXYZRGB>::Ptr cloud; //Point cloud for storing the data set
    DualSphere ranSph; //Sphere object
	sphere asphere; //Create a sphere object from object.h

	sphere sph1;//para el calculo del cilindro
	float distS1S2 = 0; //distancia entre los centros de s1 y s2
	Pnt s1;


					   //Setting the data and parameters of the algoritm
	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol) {
		cloud = cl;
		//radius = rad;
		inlierTreshold = tol;
		candidates = 1;
	}

	//Setting the data and parameters of the algoritm
	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, int iter) {
		cloud = cl;
		iterations = iter;
		inlierTreshold = tol;
		candidates = 1;
	}
	//Setting the data and parameters of the algoritm
	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, int iter, sphere sp, float dist) {
		cloud = cl;
		iterations = iter;
		inlierTreshold = tol;
		candidates = 1;
		sph1 = sp;
		s1 = Vec(sph1.x, sph1.y, sph1.z).null();
		distS1S2 = dist;

	}
	/*
	//Setting the data and parameters of the algoritm
	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, int cand) {
		cloud = cl;
		//radius = rad;
		inlierTreshold = tol;
		candidates = cand;
	}
	*/
	//Runs the primitive shape detection for a sphere
	bool compute() {
		//Variables to keep track of the algorithm
		int it = 0;
		vector<int> ran(4);
		int can = 0;
		vector<sphere> cand(candidates);
		int count;
		vector<int> inPoints(candidates);

		//Timer
		std::clock_t t1, t2;
		t1 = std::clock();


		///vars for probability stop
		double k = 3.0;
		double probability_ = 0.99;//prob. to get a selection without error
		double log_probability = log(1.0 - probability_);
		double one_over_indices = 1.0 / static_cast<double> (cloud->width);


		inPoints[0] = 0;

		//Algorithm
		//while (it < iterations) {//can < candidates &&  it < iterations) {
		while (it < k && it < iterations) {

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
				/*
				if (inPoints[0] == NULL) {
				inPoints[0] = count;
				cand[0] = asphere;
				}
				*/
				if (count > inPoints[0]) {
					cand[0] = asphere;
					inPoints[0] = count;
					can++;

					// Compute the k parameter (k=log(z)/log(1-w^n))
					double w = static_cast<double> (inPoints[0]) * one_over_indices;
					double p_no_outliers = 1.0 - pow(w, (ran.size()));// 4 puntos para crear la esfera
					p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon(), p_no_outliers);       // Avoid division by -Inf
					p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon(), p_no_outliers);   // Avoid division by 0.
					k = log_probability / log(p_no_outliers);


				}
			}
			it++;



		}

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

			//cand[best].radius = sqrt(-2 * cand[best].dualSphere[4]);
			asphere = cand[best];
			ranSph = cand[best].dualSphere;
			rad = cand[best].calcRadius(cand[best].dualSphere);


			delete indexlist;
			indexlist = new std::vector<int>;
			for (int j = 0; j < cloud->points.size(); j++) {
				if (isInlier(Vec(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z).null(), asphere)) {
					indexlist->push_back(j);
				}
			}



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

	//Runs the primitive shape detection for a sphere2 of the cylinder
	bool computeS2() {
		//Variables to keep track of the algorithm
		int it = 0;
		vector<int> ran(4);
		int can = 0;
		vector<sphere> cand(candidates);
		int count;
		vector<int> inPoints(candidates);
		Pnt s2;

		//Timer
		std::clock_t t1, t2;
		t1 = std::clock();


		///vars for probability stop
		double k = 3.0;
		double probability_ = 0.99;//prob. to get a selection without error
		double log_probability = log(1.0 - probability_);
		double one_over_indices = 1.0 / static_cast<double> (cloud->width);


		inPoints[0] = 0;

		//Algorithm
		//while (it < iterations) {//can < candidates &&  it < iterations) {
		while (it < k && it < iterations) {

			//Generate random indexes
			for (int i = 0; i<4; i++) {
				ran[i] = rand() % cloud->points.size();
			}

			//Create dual sphere with indexed points from point cloud
			asphere.defineDual(cloud, ran); //Creates a dual sphere in confromal space from 4 indexed points in a point cloud

			s2 = Vec(asphere.x, asphere.y, asphere.z).null();

			Sca d = Sca(-2)*(s1 <= s2);

											//Check if sphere is inside tolerance
			if (sphereCheck(asphere) && distS1S2 < sqrt ( abs ( d[0] ) ) ) {

				//Count inliers
				count = 0;
				for (int j = 0; j < cloud->points.size(); j++) {
					if (isInlier(Vec(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z).null(), asphere)) {
						count++;
					}
				}
				/*
				if (inPoints[0] == NULL) {
				inPoints[0] = count;
				cand[0] = asphere;
				}
				*/
				if (count > inPoints[0]) {
					cand[can] = asphere;
					inPoints[can] = count;
					can++;

					// Compute the k parameter (k=log(z)/log(1-w^n))
					double w = static_cast<double> (inPoints[0]) * one_over_indices;
					double p_no_outliers = 1.0 - pow(w, (ran.size()));// 4 puntos para crear la esfera
					p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon(), p_no_outliers);       // Avoid division by -Inf
					p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon(), p_no_outliers);   // Avoid division by 0.
					k = log_probability / log(p_no_outliers);


				}
			}
			it++;



		}

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

			//cand[best].radius = sqrt(-2 * cand[best].dualSphere[4]);
			asphere = cand[best];
			ranSph = cand[best].dualSphere;
			rad = cand[best].calcRadius(cand[best].dualSphere);


			delete indexlist;
			indexlist = new std::vector<int>;
			for (int j = 0; j < cloud->points.size(); j++) {
				if (isInlier(Vec(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z).null(), asphere)) {
					indexlist->push_back(j);
				}
			}



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


private:

	//Function to check if a point is classified as a inlier
	bool isInlier(Pnt point, sphere can) {
		Sca dist = Sca(2)*(point <= can.dualSphere) ; // 2 (P . S*)
		if ((abs(dist[0]) < inlierTreshold) && (abs(dist[0]) > -1* inlierTreshold)) {
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
};

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
		vector<int> ran(3);
		int count;
		vector<Pnt> cand(candidates);
		plane aplane; //Create a plane object from object.h
		int can = 0;
		int it = 0;
		vector<int> inPoints(candidates);


		///vars for probability stop
		double k = 3.0;
		double probability_ = 0.99;//prob. to get a selection without error
		double log_probability = log(1.0 - probability_);
		double one_over_indices = 1.0 / static_cast<double> (cloud->width);

		inPoints[0] = 0;
					  //Algorithm
		//while (it < iterations) {
		while (it < k && it < iterations) {

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
			/*
			if (inPoints[0] == NULL) {
				inPoints[0] = count;
			}
			*/
			if (count > inPoints[0]) {
				cand[0] = aplane.normDualPlane;
				inPoints[0] = count;
				can++;


				// Compute the k parameter (k=log(z)/log(1-w^n))
				double w = static_cast<double> (inPoints[0]) * one_over_indices;
				double p_no_outliers = 1.0 - pow(w, (ran.size()));// 4 puntos para crear la esfera
				p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon(), p_no_outliers);       // Avoid division by -Inf
				p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon(), p_no_outliers);   // Avoid division by 0.
				k = log_probability / log(p_no_outliers);

			}
			
			it++;
		}

		if (can > 0) {
			//Find candidate with most inliers
			int best = 0;
			int numBest = 0;
			for (int i = 0; i < cand.size(); i++) {
				if (inPoints[i] > numBest) {
					best = i;
					numBest = inPoints[i];
				}
			}
			
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

private:

	//Function to check if a point is classified as a inlier
	bool isInlier(Pnt point, Pnt plane) {
		Sca distance = point <= plane;
		if (abs(distance[0]) < planetol) {
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

//Class for detecting a cylinder in a point cloud using the sphere-sphere approach.
class ransacCylinder2{
public:
	//Constructor
	ransacCylinder2() {
			indexlist = new std::vector<int>;
	}
    PointCloud<PointXYZRGB>::Ptr cloud; //Point cloud for storing the data set
    PointCloud<PointXYZRGB>::Ptr inlierCloud; //Point cloud for storing the inliers
    float tol = 0; //Float for the radius tolerance
    float searchRadius = 0; //Float for storing the radius
    int inliers; //Integer for storing the number of inliers
    cylinder ranCyl; //Cylinder object
    ransacSphere sph; //Sphere detection object
	std::vector<int> *indexlist; //Vector for holding the index of inlier points
	int candidates = 1;
	int it=0;
	int iterations = 10;
	int count;
	std::vector<int> ran;
	int can = 0;
	int numInliers;



						 //Setting the data and parameters of the algoritm
	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tolerance) {
		cloud = cl;
		tol = tolerance;
		//searchRadius = rad;
	}

	//Runs the primitive shape detection for a cylinder using the sphere-sphere approach
	bool compute() {

		vector<int> ran(4);
		vector<cylinder> cand(candidates);
		vector<int> inPoints(candidates);
		//Variables to keep track of the algorithm
		inlierCloud.reset(new PointCloud<PointXYZRGB>);
		int icount=0;
		Pnt p1, p2;
		float p1rad, p2rad;
		sphere s1, s2;
		ransacSphere sph;


		///vars for probability stop
		double k = 300.0;
		double probability_ = 0.9;//prob. to get a selection without error
		double log_probability = log(1.0 - probability_);
		double one_over_indices = 1.0 / static_cast<double> (cloud->width);



		inPoints[0] = 0;
		//Algorithm
		//while (it < iterations) {
		while (it < k && it < iterations) {



			
			
			
			//se calcula ransac para la esfera en agc
			sph.setData(cloud, 0.005);
			sph.compute();
			s1 = sph.asphere;
			//se calcula ransac para la esfera en agc
			sph.setData(cloud, 0.005);
			sph.compute();
			s2 = sph.asphere;
			

			searchRadius = (s1.radius + s2.radius) / 2;

			//Find number of inlier points for each candidate
			count = 0;
			for (int j = 0; j < cloud->points.size(); j++) {
				if (isInlier2(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z,s1.dualSphere, s2.dualSphere, searchRadius)) {
					count++;
				}
			}

			ranCyl.defineCylinder(s1, s2, searchRadius);
			/*
			if (inPoints[0] == NULL) {
				cand[0] = ranCyl;
				inPoints[0] = count;
				can++;
			}
			*/
			if (count > inPoints[0]) {
				cand[0] = ranCyl;
				inPoints[0] = count;
				can++;


				// Compute the k parameter (k=log(z)/log(1-w^n))
				double w = static_cast<double> (inPoints[0]) * one_over_indices;
				double p_no_outliers = 1.0 - pow(w, (8));// 4 puntos para crear la esfera 8 para el cilindro
				p_no_outliers = (std::max) (std::numeric_limits<double>::epsilon(), p_no_outliers);       // Avoid division by -Inf
				p_no_outliers = (std::min) (1.0 - std::numeric_limits<double>::epsilon(), p_no_outliers);   // Avoid division by 0.
				k = log_probability / log(p_no_outliers);
			}

			it++;
		}




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

			ranCyl = cand[best];



			delete indexlist;
			indexlist = new std::vector<int>;
			for (int j = 0; j < cloud->points.size(); j++) {
				if (isInlier2(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z, ranCyl.s1.dualSphere, ranCyl.s2.dualSphere, ranCyl.radius)) {
					indexlist->push_back(j);
				}
			}



			return true;
		}
		else {
			//Return false if not
			return false;
		}





	}


private:
	//Function to check if a point is classified as a inlier
	bool isInlier2(float x, float y, float z, Pnt sd1, Pnt sd2, float radius) {
		Pnt p1 = Vec(x, y, z).null();
		Pnt ni = Vec().null();

		float rMax, rMin;

		rMax = radius + tol;
		rMin = radius - tol;

		ni[0] = 0;
		ni[1] = 0;
		ni[2] = 0;
		ni[3] = 0;
		ni[4] = 1;
		Par ppar = sd1 ^ sd2;
		Lin L = ni ^ ppar  ;

		DualSphere s= ((p1 ^ L.dual())/L.dual());
		Sca d = ( s <= s);
		if ( ( abs(d[0]) > ( rMin)*(rMin)) && ( abs(d[0]) < (rMax)*(rMax))) {
			return true;
		}
		else {
			return false;
		}
	}

};


 