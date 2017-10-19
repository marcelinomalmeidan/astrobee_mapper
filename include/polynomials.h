#ifndef _H_POLYNOMIAL_CLASSES_
#define _H_POLYNOMIAL_CLASSES_

#include <iostream>
#include <Eigen/Dense>
#include <complex>
#include "astrobee_mapper/ControlGoal.h"
#include "astrobee_mapper/ControlState.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


//Coefficients go from higher order to lower order
//e.g.: (t-t0)^2 + 2(t-t0) + 3 ==> coeff = [1 2 3]
class polynomial{
public:
	double t0;				//Initial time for definition of polynomial
	double tf;				//Final time for definition of polynomial
	int order; 				//Polynomial order
	Eigen::VectorXd coeff; 	//Coefficients

	//Constructor
	polynomial(const double t0_in, 
		       const double tf_in, 
		       const int coeffSize);
	polynomial();
	
	//Methods
	void printPolyCoeff();						//Print all coefficients
	void polyConv(const polynomial *poly2,
		                polynomial *polyOut);	//Convolute with another polynomial
	void polySquare(polynomial *polyOut);		//Square a polynomial
	void polyDiff(polynomial *polyOut);			//Get the derivative of a polynomial
	void polyAtTime(const double time,
	                double &result); 			//Return polynomial value at given time
	std::vector<std::complex<double>> roots2ndOrderPoly();
	std::vector<std::complex<double>> roots3rdOrderPoly();

};


//3D trajectories characterized by three polynomials 
class poly_3D{
public:	
	polynomial polyX;
	polynomial polyY;
	polynomial polyZ;
	double t0;				//Initial time for definition of polynomials
	double tf;				//Final time for definition of polynomials
	int order; 				//Polynomial order

	//Constructor: define a polynomial from a "ControlState" msg type (2nd order polynomials only)
	poly_3D(const double t0_in, 
		    const double tf_in, 
		    const astrobee_mapper::ControlState segment);
	poly_3D();

	//Methods
	void printSegmentCoeff();						//Print all coefficients
	void segmentAtTime(const double time,
	                   Eigen::Vector3d &result);	//Return 3d value for polynomials at a given time
	void segmentAtTime(const double time,
	                   pcl::PointXYZ &result);

};

//3D trajectories characterized by a set of 3D polynomials
class trajectory_3D{
public:
	std::vector<poly_3D> segments_poly;
	double t0;	//Initial time for the first segment
	double tf;	//Final time for the last segment
	int nSegments;

	//Constructor
	trajectory_3D(const astrobee_mapper::ControlGoal segments);
	
	//Methods
	void printTrajCoeff();							//Print coefficients from all segments
	void trajectoryAtTime(const double time,
	                      Eigen::Vector3d &result);	//Return 3d value for polynomials at a given time
	void trajectoryAtTime(const double time,
	                      pcl::PointXYZ &result);	//Return 3d value for polynomials at a given time
};



#endif