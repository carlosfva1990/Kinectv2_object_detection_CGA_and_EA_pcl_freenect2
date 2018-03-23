//
// Created by Aksel Sveier. Spring 2016
//

#ifndef RANSACGA_OBJECTS_H
#define RANSACGA_OBJECTS_H

#endif //RANSACGA_OBJECTS_H

#include <space/vsr_cga3D_op.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

using namespace vsr;
using namespace vsr::cga;
using namespace pcl;


//Class that creates and handles spheres with CGA
class sphere{
public:
    float radius, x, y, z; //Floats for storing the parameters of the sphere
    Pnt dualSphere; //Stores the IPNS representation
    Sph sphe; //Stores the OPNS representation

    void defineDual(float x, float y, float z, float r); //Creates a sphere from center(x,y,z) coordinates and radius
    void defineDual(Pnt p1, Pnt p2, Pnt p3, Pnt p4); //Creates a sphere from 4 points in conformal space.
    void defineDual(PointCloud<PointXYZRGB>::Ptr cl1, PointCloud<PointXYZRGB>::Ptr cl2, PointCloud<PointXYZRGB>::Ptr cl3, PointCloud<PointXYZRGB>::Ptr cl4); //Creates a sphere from 4 points from a pointcloud.
    void defineDual(PointCloud<PointXYZRGB>::Ptr cl, vector<int> index); //Creates a sphere in from 4 indexed points in a point cloud
private:
    float calcRadius(Pnt dSph); //Function that calculates radius
    Pnt normalize(Pnt dSph); //Function that normalizes a conformal vector
};

//Class that creates and handles planes with CGA
class plane
{
public:
    Pnt dualPlane; //Stores the IPNS representation
    Pnt normDualPlane; //Stores the normalized IPNS representation

    void defineDual(Pnt p1, Pnt p2, Pnt p3); //Creates a plane from 3 points in conformal space
    void defineDual(PointCloud<PointXYZRGB>::Ptr cl, int *index); //Creates a plane from 3 indexed points in a point cloud
private:
    void normalize(Pnt dPln); //Function that normalizes a conformal vector
};

//Class that creates and handles circles with CGA
class circle
{
public:
    float radius = 0; //Float for storing the circle radius
    Pnt circleCenter; //Point in conformal space for storing the circle center
    Par circle; //Store the OPNS representation
    Cir dualCircle; //Stores the IPNS representation
    Pnt plane; //Stores the plane the the circle lie on

    void defineCircle(Pnt p1, Pnt p2, Pnt p3); //Creates circle from 3 points in conformal space
    void defineCircle(PointXYZRGB p1, PointXYZRGB p2, PointXYZRGB p3); //Creates a circle from 3 point from a point cloud
    void defineCircle(PointCloud<PointXYZRGB>::Ptr cl, int *index); //Creates circle from 3 indexed points in a point cloud

private:
void calcRadius(Pnt p); //Function that finds the cricle radius
void findNormal(Pnt pln); //Creates the plane that the cricle lie on
};

//Class that creates and handles cylinders with CGA
class cylinder{
//Since there is no representation of a cylinder in CGA a cylinder will be represente by a circle on a plane.
//The direction of the normal of the plane in the circle center will be the center-axis of the cylinder
public:
    Pnt plan; //Store the IPNS representation of a plane
    Pnt center; //Stores the center in a conformal vector
    float radius = 0; //Float for storing the radius

    void defineCylinder(Pnt p1, Pnt p2, float rad); //Creates a cylinder from two points in conformal space on the center axis and a radius
    void defineCylinder(Pnt p1, Pnt p2, Pnt p3); //Creates a cylinder from 3 points in conformal space
    void defineCylinder(PointCloud<PointXYZRGB>::Ptr cl, int *index); //Creates the cylinder from 3 indexed points in a point cloud
};

