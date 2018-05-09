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


	//Setting the data and parameters of the algoritm
	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol) {
		cloud = cl;
		//radius = rad;
		inlierTreshold = tol;
		candidates = 1;
	}

	//Setting the data and parameters of the algoritm
	void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, int cand) {
		cloud = cl;
		//radius = rad;
		inlierTreshold = tol;
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
			rad = sqrt(-2*ranSph[4]);
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
		/*
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
		*/
		return sph.dualSphere;
	}

private:

	//Function to check if a point is classified as a inlier
	bool isInlier(Pnt point, sphere can) {
		Sca dist = Sca(2)*(point <= can.dualSphere) ; // 2 (P . S*)
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
			//Find candidate with most inliers
			int best = 0;
			int numBest = 0;
			for (int i = 0; i < cand.size(); i++) {
				if (inPoints[i] > numBest) {
					best = i;
					numBest = inPoints[i];
				}
			}
			
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
	/*
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
	}*/

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
	int iterations = 1000;
	int count;
	std::vector<int> ran;
	int can = 0;
	int numInliers;

    //Length
    //Eigen::Vector4f centroid; //Centroid of inliers
    //Eigen::Vector3f projL; //Point at the edge of cylinder on the cylinder axis
    //Eigen::Vector3f projS; //Point at the edge of cylinder on the cylinder axis
    int largestInd = 0; //Index of point at cylinder edge
    int smallestInd = 0; //Index of point at cylinder edge

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

		//Algorithm
		while (it < iterations) {

			//Generate random indexesx
			for (int i = 0; i<4; i++) {
				ran[i] = rand() % cloud->points.size();
			}

			s1.defineDual(cloud, ran);

			//Generate random indexesx
			for (int i = 0; i<4; i++) {
				ran[i] = rand() % cloud->points.size();
			}

			s2.defineDual(cloud, ran);

			searchRadius = (s1.radius + s2.radius) / 2;

			//Find number of inlier points for each candidate
			count = 0;
			for (int j = 0; j < cloud->points.size(); j++) {
				if (isInlier2(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z,s1.dualSphere, s2.dualSphere)) {
					count++;
				}
			}

			ranCyl.defineCylinder(s1.dualSphere, s2.dualSphere, searchRadius);

			if (inPoints[0] == NULL) {
				inPoints[0] = count;
			}

			if (count > inPoints[0]) {
				cand[0] = ranCyl;
				inPoints[0] = count;
				can++;
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
				if (isInlier2(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z, ranCyl.s1.dualSphere, ranCyl.s2.dualSphere)) {
					indexlist->push_back(j);
				}
			}



			return true;
		}
		else {
			//Return false if not
			return false;
		}













		/*

		ransacSphere ball;

		
		//Find the first sphere that satisfies the radius
		ball.setData(cloud, tol); //Ransac for spheres, data is set
		ball.compute();
		p1 = ball.ranSph;
		p1rad = ball.rad;

		//Find the second sphere that satisfies the radius
		ball.setData(cloud, tol);
		ball.compute();
		p2 = ball.ranSph ;
		p2rad = ball.rad;


		
		indexlist = new std::vector<int>;

		//Count the inliers for the cylinder and store the inliers in a inlier cloud
		for (int i = 0; i < cloud->points.size(); i++) {
			if (isInlier2(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, p1, p2)) {
				//inlierCloud->push_back(cloud->points[i]); 
				indexlist->push_back(i);
				icount++;
			}
		}
		inliers = icount;
		return true;
		*/
	}
	/*   
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
	*/
private:
	//Function to check if a point is classified as a inlier
	bool isInlier2(float x, float y, float z, Pnt sd1, Pnt sd2) {
		Pnt p1 = Vec(x, y, z).null();
		Pnt ni = Vec().null();
		ni[0] = 0;
		ni[1] = 0;
		ni[2] = 0;
		ni[3] = 0;
		ni[4] = 1;
		Par ppar = sd1 ^ sd2;
		Lin L = ni ^ ppar  ;

		Sca d = Sca(2)*(p1 <= ((p1 ^ L.dual()) / L.dual()));
		if ( ( d[0] <  tol ) && ( d[0] > -1 * tol)) {
			return true;
		}
		else {
			return false;
		}
	}

};


 