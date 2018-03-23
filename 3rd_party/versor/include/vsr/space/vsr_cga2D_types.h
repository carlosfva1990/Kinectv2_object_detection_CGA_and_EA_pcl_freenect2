#ifndef VSR_CGA2D_TYPES_INCLUDED
#define VSR_CGA2D_TYPES_INCLUDED

#include "detail/vsr_multivector.h"
#include "detail/vsr_generic_op.h"    

namespace vsr{ namespace cga2D { 
  

    //2D CONFORMAL 
   // typedef Ne<4,1> e1; 
   // typedef Ne<4,2> e2; 
   // typedef Ne<4,3> e3; 
    
    typedef NSca<4> Sca; 
    typedef NVec<4> Vec; 
    typedef NBiv<4> Biv; 
    typedef NTri<4> Tri; 
    typedef NRot<4> Rot;
    
    typedef NOri<4> Ori;   //Origin
    typedef NInf<4> Inf;   //Infinity
    typedef NMnk<4> Mnk;   //E Plane
    typedef NPss<4> Pss;   //E Plane
    
    typedef NPnt<4> Pnt;   //Homogenous Point in 3D  
    typedef Pnt Dls; 		   //Dual Sphere    
    
    typedef NPar<4> Par;   //Point Pair
    typedef NCir<4> Cir;	 //Circle
    typedef NSph<4> Sph;	 //Sphere
                        	
    typedef NDrv<4> Drv;	 //Direction Vector
    typedef NTnv<4> Tnv;	 //Tangent Vector   
    typedef NDrb<4> Drb;	 //Direction Bivector
    typedef NTnb<4> Tnb;	 //Tangent Bivector  
    typedef NDrt<4> Drt;	 //Direction Trivector
    typedef NTnt<4> Tnt;	 //Tangent Trivector  
                        	
    typedef NDll<4> Dll;	 //Dual Line        
    typedef NLin<4> Lin;	 //Dual Line    
    typedef NFlp<4> Flp;	 //Flat Point 
    typedef decltype( Flp().dual() ) Dfp; //Dual Flat Point
    typedef NPln<4> Pln;	 //Plane 
    typedef NDlp<4> Dlp;	 //Plane   
                        	
    typedef NTrs<4> Trs;	 //Translator 
    typedef NMot<4> Mot;	 //Motor 
    typedef NTrv<4> Trv;	 //Transversor 
    typedef NBst<4> Bst;	 //Boost 
    typedef NDil<4> Dil;	 //Dilator 
    typedef NTsd<4> Tsd;	 //Translated Dilator  

    //FULL NAMES
    typedef  Sca Scalar;
    typedef  Biv Bivector;
    typedef  Tri Trivector ;
    typedef  Rot Rotor;
    typedef  Ori Origin;
    typedef  Inf Infinity;
    typedef  Mnk Minkowski;
    typedef  Pss Pseudoscalar;
    typedef  Pnt Point;
    typedef  Par Pair;
    typedef  Cir Circle;
    typedef  Sph Sphere;
    typedef  Pnt DualSphere;
    typedef  Drv DirectionVector;
    typedef  Drb DirectionBivector;
    typedef  Drt DirectionTrivector;
    typedef  Tnv TangentVector;
    typedef  Tnb TangentBivector;
    typedef  Tnt TangentTrivector;
    typedef  Dll DualLine;
    typedef  Lin Line;
    typedef  Flp FlatPoint;
    typedef  Dfp DualFlatPoint;
    typedef  Pln Plane;
    typedef  Dlp DualPlane;
    typedef  Trs Translator;
    typedef  Mot Motor;
    typedef  Trv Transversor;
    typedef  Bst Boost;
    typedef  Dil Dilator;
    typedef  Tsd TranslatedDilator;

   //move to Construct::                   
 //   template<class V>
 //   inline Point point(const V& v){
 //     return nga::Round::null( v[0], v[1]  );
 //   }
 // 
 //   Point circle(VSR_PRECISION x, VSR_PRECISION y, VSR_PRECISION rad){
 //     return nga::Round::dls( Vec(x,y), rad ); 
 //   }

 //   Pair meet( const Line& lin, const Circle& cir){
 //     return ( lin.dual() ^ cir.dual() ).dual();
 //   }

} } // vsr::cga2D

#endif
