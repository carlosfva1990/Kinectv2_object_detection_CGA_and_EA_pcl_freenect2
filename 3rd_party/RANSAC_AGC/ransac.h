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
    naiveSphere();
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



    void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, float rad, int inliers, double rat);
    void compute(); //does all computation, and eventually stores the sphere with the most inliers as a Pnt type;
    sphere fit(sphere sph);

private:
    bool isInlier(float x, float y, float z, sphere can);
    bool sphereCheck(sphere sph);
};

class naiveSphere2{
public:
    naiveSphere2();
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



    void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, int inliers, double rat);
    void compute(); //does all computation, and eventually stores the sphere with the most inliers as a Pnt type;
    sphere fit(sphere sph);

private:
    bool isInlier(float x, float y, float z, sphere can);
};


//Class for detecting a sphere in a point cloud with ransac
class ransacSphere{
public:
    ransacSphere(); //Constructor
    float radiusTolerance;
    float inlierTreshold = 0.005; //For the kinect the accuracy is 1 % of the distance from the sensor for 100 images.
    int iterations = 1; //Maximum allowed iterations
    int actualIt; //Integer for storing the number of iterations performed
    float radius = 0; // float for storing the raduis
    PointCloud<PointXYZRGB>::Ptr cloud; //Point cloud for storing the data set

    float time; //Float for storing the computation time of the algorithm

    Pnt ranSph; //Sphere object
    int candidates = 1; //Number of candidates to detect
    std::vector<int> *indexlist; //Vector for storing inliers
    int numInliers; //Integer for storing the number of inliers

    void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, float rad);
    void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, float rad, int cand);
    bool compute(); //does all computation, and eventually stores the sphere with the most inliers as a Pnt type;
    Pnt fit(sphere sph); //Sphere fitting in conformal space.

private:
    bool isInlier(float x, float y, float z, sphere can); //Bool for determining whether a point is a inlier
    bool sphereCheck(sphere sph); //Check if the sphere radius is inside tolerance limits
    int estimateInliers(sphere sph); //Function for estimating the inliers for a specifc sphere.
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

/////////////////////// MULTIPLE SPHERES  ////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
class ransacSpheres{
public:
    ransacSpheres(); //Constructor
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


    void setData(PointCloud<PointXYZRGB>::Ptr cl, float resolution);
    void setOctreeBounds(double xmin, double ymin, double zmin, double xmax, double ymax, double zmax); //Function for setting the octree bounds
    void setOctree(float resolution); //Function for creating the point picking octree with a desired resolution
    void setSearchOctree(float resolution); //Function for creating the search octree with a desired resolution
    void compute(); //Function that runs the algorithm
    sphere fit(sphere sph, pcl::PointIndices::Ptr inliers); //Function for fitting a sphere to its inliers
    void divideCloud();

private:
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGB>::Ptr octSearch; //Search octree object
    pcl::octree::OctreePointCloud<pcl::PointXYZRGB>::Ptr oct; //Point picking octree object

    vector<int> shuffle(vector<int> list); //Function for shuffeling a index vector
    int getInliers(sphere can); //Function for counting the inliers for a sphere from a subcloud
    bool isInlier(float x, float y, float z, sphere can); //Function for deremining whether a point is a inlier
    int estimateInliers(sphere sph); //Function for estimating the inliers of a sphere
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////      PLANE     ////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Class for detecting a plane in a point cloud with RANSAC
class ransacPlane{
public:
    ransacPlane(); //Constructor
    float planetol; //Tolerance used for deciding wether a point is compatable with a plane
    PointCloud<PointXYZRGB>::Ptr cloud; //Point cloud for holding the data set
    PointCloud<PointXYZRGB>::Ptr segment; //Point cloud for holding the inlier points
    std::vector<int> *indexlist; //Vector for holding the index of inlier points

    Pnt ranPln; //Plane object
    int candidates; //Number of candidates to detect

    void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, int cand);
    void compute(); //does all computation, and eventually stores the plane with the most inliers as a Pnt type;
    void segmentPlane(PointCloud<PointXYZRGB>::Ptr seg); //Function that creates and stores the indexlist of the current plane
    Pnt fit(Pnt pln); //Plane fitting in conformal space
private:
    bool isInlier(Pnt point, Pnt plane); //Bool for determining weather a point is a inlier
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

    void setData(PointCloud<PointXYZRGB>::Ptr cl, float tol, float rad);
    void compute(); //does all computation, and eventually stores the cricle with most inliers
private:
    bool isInlier(float x, float y, float z, Pnt center, Pnt plane);
    bool radiusCheck(float rad);
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


    void setData(PointCloud<PointXYZRGB>::Ptr cl, float tolerance, float rad, int inliers, int cand);
    void compute(); //does all computation, and eventually stores the cylinder with most inliers
    void cylinderLength(); //Function for determining the cylinder length
private:
    bool isInlier(float x, float y, float z, Pnt p1, Pnt p2); //Bool for determining weather a point is a inlier
    bool radiusCheck(float rad); //Check if the cylinder radius is inside tolerance limits
};

//Class for detecting a cylinder in a point cloud using the sphere-sphere approach.
class ransacCylinder2{
public:
    ransacCylinder2(); //Constructor
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

    void setData(PointCloud<PointXYZRGB>::Ptr cl, float tolerance, float rad);
    void compute(); //does all computation, and eventually stores the cylinder with most inliers
    void cylinderLength(cylinder cyl); //Function for determining the cylinder length
private:
    bool isInlier2(float x, float y, float z, Pnt p1, Pnt p2); //Bool for determining weather a point is a inlier
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

    void setData(PointCloud<PointXYZRGB>::Ptr cl, float tolerance, float rad, int inliers, double rat);
    void compute(); //does all computation, and eventually stores the cricle with most inliers

    //Lengrth
    Eigen::Vector4f centroid;

    Eigen::Vector3f projL;
    Eigen::Vector3f projS;

    int largestInd = 0;
    int smallestInd = 0;
    void cylinderLength();
private:
    bool isInlier(float x, float y, float z, Pnt p1, Pnt p2);
    bool radiusCheck(float rad);
};

