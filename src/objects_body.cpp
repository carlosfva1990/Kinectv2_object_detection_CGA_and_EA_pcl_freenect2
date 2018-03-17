//
// Created by Aksel Sveier. Spring 2016
//

#include "objects.h"
#include <vsr/space/vsr_cga3D_op.h>
#include <pcl/point_types.h>
#include <pcl/common/common_headers.h>

using namespace vsr;
using namespace vsr::cga;
using namespace pcl;

//Creates a sphere from center(x,y,z) coordinates and radius
void sphere::defineDual(float x, float y, float z, float r){
    dualSphere = normalize(Round::dls( Vec( x,y,z ), r ));
    radius = r;
}

//Creates a sphere from 4 points in conformal space.
void sphere::defineDual(Pnt p1, Pnt p2, Pnt p3, Pnt p4){
    dualSphere = normalize((p1^p2^p3^p4).dual());
    radius = calcRadius(dualSphere);
}

//Creates a sphere from 4 points from a pointcloud.
void sphere::defineDual(PointCloud<PointXYZRGB>::Ptr cl1, PointCloud<PointXYZRGB>::Ptr cl2, PointCloud<PointXYZRGB>::Ptr cl3, PointCloud<PointXYZRGB>::Ptr cl4){
    dualSphere = normalize((Vec(cl1->points[0].x,cl1->points[0].y,cl1->points[0].z).null()
     ^ Vec(cl2->points[0].x,cl2->points[0].y,cl2->points[0].z).null()
     ^ Vec(cl3->points[0].x,cl3->points[0].y,cl3->points[0].z).null()
     ^ Vec(cl4->points[0].x,cl4->points[0].y,cl4->points[0].z).null()).dual());
    radius = calcRadius(dualSphere);
}

//Creates a sphere in from 4 indexed points in a point cloud
void sphere::defineDual(PointCloud<PointXYZRGB>::Ptr cl, vector<int> index){
    dualSphere = normalize((Vec(cl->points[index[0]].x,cl->points[index[0]].y,cl->points[index[0]].z).null()
                  ^ Vec(cl->points[index[1]].x,cl->points[index[1]].y,cl->points[index[1]].z).null()
                  ^ Vec(cl->points[index[2]].x,cl->points[index[2]].y,cl->points[index[2]].z).null()
                  ^ Vec(cl->points[index[3]].x,cl->points[index[3]].y,cl->points[index[3]].z).null()).dual());
    radius = calcRadius(dualSphere);
    x = dualSphere[0]; y = dualSphere[1]; z = dualSphere[2];
}

//Function that calculates radius
float sphere::calcRadius(Pnt sph){
    return sqrt((1/pow(sph[3],2))*(pow(sph[0],2)+pow(sph[1],2)+pow(sph[2],2)) - (2*sph[4]/sph[3]));
}

//Function that normalizes a conformal vector
Pnt sphere::normalize(Pnt dSph){
    Pnt ret;
    for(int i = 0; i<5;i++){
        ret[i] = dSph[i]/dSph[3];
    }
    return ret;
}

//Creates a plane from 3 points in conformal space
void plane::defineDual(Pnt p1, Pnt p2, Pnt p3){
    dualPlane = (p1^p2^p3^Inf(1)).dual();
    normalize(dualPlane);
}

//Creates a plane from 3 indexed points in a point cloud
void plane::defineDual(PointCloud<PointXYZRGB>::Ptr cl, int *index){
    dualPlane = (Vec(cl->points[index[0]].x,cl->points[index[0]].y,cl->points[index[0]].z).null()
                            ^ Vec(cl->points[index[1]].x,cl->points[index[1]].y,cl->points[index[1]].z).null()
                            ^ Vec(cl->points[index[2]].x,cl->points[index[2]].y,cl->points[index[2]].z).null()
                          ^ Inf(1)).dual();
    normalize(dualPlane);
}

//Function that normalizes a conformal vector
void plane::normalize(Pnt dPln){
    for(int i = 0; i < 5; i++){
        normDualPlane[i] = dPln[i]/sqrt(pow(dPln[0],2)+pow(dPln[1],2)+pow(dPln[2],2));
    }
}

//Creates circle from 3 points in conformal space
void circle::defineCircle(Pnt p1, Pnt p2, Pnt p3){
    dualCircle = p1 ^ p2 ^ p3;
    circle = (p1 ^ p2 ^ p3).dual();
    Pnt temp = ((p1 ^ p2 ^ p3).dual()) * Inf(1) * ((p1 ^ p2 ^ p3).dual());
    for(int i= 0; i<5; i++){
        circleCenter[i] = temp[i]/temp[3];
    }
    findNormal((p1 ^ p2 ^ p3 ^ Inf(1)).dual());
    calcRadius(p1);
}

//Creates a circle from 3 point from a point cloud
void circle::defineCircle(PointCloud<PointXYZRGB>::Ptr cl, int *index){
    dualCircle = Vec(cl->points[index[0]].x, cl->points[index[0]].y, cl->points[index[0]].z).null()
            ^ Vec(cl->points[index[1]].x, cl->points[index[1]].y, cl->points[index[1]].z).null()
            ^ Vec(cl->points[index[2]].x, cl->points[index[2]].y, cl->points[index[2]].z).null();
    circle = dualCircle.dual();
    Pnt temp = circle * Inf(1) * circle;
    for(int i= 0; i<5; i++){
        circleCenter[i] = temp[i]/temp[3];
    }
    findNormal((Vec(cl->points[index[0]].x, cl->points[index[0]].y, cl->points[index[0]].z).null() ^ Vec(cl->points[index[1]].x, cl->points[index[1]].y, cl->points[index[1]].z).null() ^ Vec(cl->points[index[2]].x, cl->points[index[2]].y, cl->points[index[2]].z).null() ^ Inf(1)).dual());
    calcRadius(Vec(cl->points[index[0]].x, cl->points[index[0]].y, cl->points[index[0]].z).null());
}

//Creates circle from 3 indexed points in a point cloud
void circle::defineCircle(PointXYZRGB p1, PointXYZRGB p2, PointXYZRGB p3){
    dualCircle = Vec(p1.x,p1.y,p1.z).null()
            ^ Vec(p2.x,p2.y,p2.z).null()
            ^ Vec(p3.x,p3.y,p3.z).null();
    circle = dualCircle.dual();
    Pnt temp = circle * Inf(1) * circle;
    for(int i= 0; i<5; i++){
        circleCenter[i] = temp[i]/temp[3];
    }
    findNormal((Vec(p1.x,p1.y,p1.z).null() ^ Vec(p2.x,p2.y,p2.z).null() ^ Vec(p3.x,p3.y,p3.z).null() ^ Inf(1)).dual());
    calcRadius(Vec(p1.x,p1.y,p1.z).null());
}

//Function that finds the cricle radius
void circle::calcRadius(Pnt p){
    radius = sqrt(pow(p[0] - circleCenter[0],2) + pow(p[1] - circleCenter[1],2) +pow(p[2] - circleCenter[2],2) );
}

//Function that creates the plane that the cricle lies on
void circle::findNormal(Pnt pln){
    for(int i = 0; i < 5; i++){
        plane[i] = pln[i]/sqrt(pow(pln[0],2)+pow(pln[1],2)+pow(pln[2],2));
    }
}

//Creates a cylinder from two points in conformal space on the center axis and a radius
void cylinder::defineCylinder(Pnt p1, Pnt p2, float rad){
    radius = rad;
    center = p1;
    for(int i = 0; i < 3; i++){
        plan[i] = (p1[i] - p2[i])/sqrt(pow(p1[0] - p2[0],2) + pow(p1[1] - p2[1],2) + pow(p1[2] - p2[2],2));
    }
    plan[3] = 0;
}

//Creates a cylinder from 3 points in conformal space
void cylinder::defineCylinder(Pnt p1, Pnt p2, Pnt p3){
    plane pln;
    pln.defineDual(p1,p2,p3);
    plan = pln.normDualPlane;
    circle cir;
    cir.defineCircle(p1, p2, p3);
    center = cir.circleCenter;
    radius = cir.radius;
}

//Creates the cylinder from 3 indexed points in a point cloud
void cylinder::defineCylinder(PointCloud<PointXYZRGB>::Ptr cl, int *index){
    plane pln;
    pln.defineDual(cl, index);
    plan = pln.normDualPlane;
    circle cir;
    cir.defineCircle(cl, index);
    center = cir.circleCenter;
    radius = cir.radius;
}




