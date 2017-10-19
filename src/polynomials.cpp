

#include "polynomials.h"


//Constructor method
polynomial::polynomial(const double t0_in, 
	                   const double tf_in, 
	                   const int coeffSize){
	t0 = t0_in;
	tf = tf_in;
	order = coeffSize - 1;
	coeff = Eigen::MatrixXd::Zero(coeffSize,1);
}

polynomial::polynomial(){
	t0 = 0.0;
	tf = 1.0;
	order = 2;
	coeff = Eigen::MatrixXd::Zero(1,1);
}

//Print all coefficients
void polynomial::printPolyCoeff(){
	int n_polyCoeff = order + 1;
	// std::cout << "vector length: " << n_polyCoeff << std::endl;
	// std::cout << "t0: " << poly.t0 << "\ttf: " << poly.tf << std::endl;
	for (int i = 0; i < n_polyCoeff; i++){
		std::cout << coeff(i) << " ";
	}
	std::cout << std::endl;
}

//Convolute with another polynomial
void polynomial::polyConv(const polynomial *poly2,
						  polynomial *polyOut){
	static int m, n, nCoeff_polyOut, k, j_min, j_max;
	m = order + 1;
	n = poly2->order + 1;
	nCoeff_polyOut = m + n - 1;

	for(int k = 0; k < nCoeff_polyOut; k++){
		j_min = std::max(0,k-n+1);
		j_max = std::min(k,m-1);
		for(int j = j_min; j <= j_max; j++){
			polyOut->coeff(k) = polyOut->coeff(k) + coeff(j)*poly2->coeff(k-j);
		}
	}

}

//Square a polynomial
void polynomial::polySquare(polynomial *polyOut){

	if(order == 2){ //Closed form solution
		static Eigen::VectorXd coeffOut(5);
		static double a, b, c;
		a = coeff(0);
		b = coeff(1);
		c = coeff(2);

		coeffOut << a*a, 2*a*b, 2*a*c+b*b, 2*b*c, c*c;
		polyOut->coeff = coeffOut;
	}
	else{ //Convolute the polynomial with itself
		this->polyConv(this, polyOut);
	}

	// polyOut->printPolyCoeff(); std::cout << std::endl;
}

//Get the derivative of a polynomial
void polynomial::polyDiff(polynomial *polyOut){

	static int power;
	for (int i = 0; i < order; i++){
		power = order - i;
		polyOut->coeff(i) = coeff(i)*power;
	}

}

//Return polynomial value at given time
void polynomial::polyAtTime(const double time,
	           				double &result){
	static int m, power;
	static double t;
	m = order + 1;
	result = 0.0;

	t = time - t0;
	if(t != 0){
		for (int i = 0; i < m; i++){
			power = m - i - 1;
			result = result + coeff(i)*pow(t,power);
		}
	}
	else{
		result = coeff(m-1);
	}

	// return result;
}

std::vector<std::complex<double>> polynomial::roots2ndOrderPoly(){
	
	//Set the coefficients in friendly form
	double a = coeff(0);
	double b = coeff(1);
	double c = coeff(2);

	//Calculate the quadratic formula for all cases
	double Delta = b*b - 4*a*c;
	std::vector<std::complex<double>> solution;
	std::complex<double> sol1, sol2;
	if(Delta > 0){ //There are two distinct real roots
		sol1 = std::complex<double>( (-b+sqrt(Delta))/(2*a) , 0.0);
		sol2 = std::complex<double>( (-b-sqrt(Delta))/(2*a) , 0.0);
	}
	else if(Delta == 0){ //There are two real multiple roots
		sol1 = std::complex<double>( -b/(2*a) , 0.0);
		sol2 = std::complex<double>( -b/(2*a) , 0.0);
	}
	else{ //There are two complex roots
		sol1 = std::complex<double>( -b/(2*a) ,  sqrt(-Delta)/(2*a));
		sol2 = std::complex<double>( -b/(2*a) , -sqrt(-Delta)/(2*a));
	}
	solution.push_back(sol1);
	solution.push_back(sol2);

	// std::cout << solution[0] << std::endl << 
	//              solution[1] << std::endl;

	return solution;
}

//Find the roots of a 3rd order polynomial
//Algebraic solution from https://en.wikipedia.org/wiki/Casus_irreducibilis
std::vector<std::complex<double>> polynomial::roots3rdOrderPoly(){
	
	//Set the coefficients in friendly form
	static double a, b, c, d;
	a = coeff(0);
	b = coeff(1);
	c = coeff(2);
	d = coeff(3);

	//Get Cardano's coefficients
	static double p, q;
	p = (3*a*c - b*b)/(3*a*a);
	q = (2*b*b*b - 9*a*b*c + 27*a*a*d)/(27*a*a*a);

	//Cube roots of 1
	std::vector<std::complex<double>> w;
	w.push_back(std::complex<double>(1.0,0.0));
	w.push_back(std::complex<double>(-0.5,0.5*sqrt(3)));
	w.push_back(std::complex<double>(-0.5,-0.5*sqrt(3)));

	//Implementation of the solution
	static double Delta, term, term1, term2;
	std::vector<std::complex<double>> solution;
	Delta = q*q/4.0 + p*p*p/27.0;
	term = b/(3.0*a);
	if(Delta >= 0.0){ //Casus irreducibilis
		term1 = -q/2.0 + sqrt(Delta);
		term2 = -q/2.0 - sqrt(Delta);

		//Calculate the three solutions
		for(int i = 0; i < 3; i++){
			std::complex<double> t = w[i]*cbrt(term1) + w[i]*w[i]*cbrt(term2);
			solution.push_back(t - term);
		}
	}
	else{ //Trigonometric solution
		term1 = acos(3.0*q*sqrt(-3.0/p)/(2.0*p));
		for(int i = 0; i < 3; i++){
			std::complex<double> t = 2*sqrt(-p/3.0)*cos(term1/3.0 - double(i)*2*M_PI/3.0);
			solution.push_back(t - term);
			// std::cout << t.real() << std::endl;
		}
	}

	return solution;
}


//Constructor: define a polynomial from a "ControlState" msg type (2nd order polynomials only)
poly_3D::poly_3D(const double t0_in, 
	             const double tf_in, 
	             const astrobee_mapper::ControlState segment){
	int n_coeff = 3;
	polyX.coeff.resize(n_coeff);
	polyY.coeff.resize(n_coeff);
	polyZ.coeff.resize(n_coeff);
	polyX = polynomial(t0_in,tf_in,n_coeff);
	polyY = polynomial(t0_in,tf_in,n_coeff);
	polyZ = polynomial(t0_in,tf_in,n_coeff);
	t0 = t0_in;
	tf = tf_in;
	order = n_coeff - 1;

	//Set the polynomial coefficients
	Eigen::VectorXd coeffX(n_coeff), coeffY(n_coeff), coeffZ(n_coeff);
	coeffX << 0.5*segment.accel.linear.x,
	          segment.twist.linear.x,
	          segment.pose.position.x;
	coeffY << 0.5*segment.accel.linear.y,
	          segment.twist.linear.y,
	          segment.pose.position.y;
	coeffZ << 0.5*segment.accel.linear.z,
	          segment.twist.linear.z,
	          segment.pose.position.z;
	polyX.coeff = coeffX;
	polyY.coeff = coeffY;
	polyZ.coeff = coeffZ;
}

poly_3D::poly_3D(){
	int n_coeff = 1;
	double t0_in = 0.0;
	double tf_in = 1.0;
	polyX.coeff.resize(n_coeff);
	polyY.coeff.resize(n_coeff);
	polyZ.coeff.resize(n_coeff);
	polyX = polynomial(t0_in,tf_in,n_coeff);
	polyY = polynomial(t0_in,tf_in,n_coeff);
	polyZ = polynomial(t0_in,tf_in,n_coeff);
	t0 = t0_in;
	tf = tf_in;
}

void poly_3D::printSegmentCoeff(){
	    std::cout << "t0: " << t0 << "\ttf: " << tf <<  std::endl;
		std::cout << "polyX: "; polyX.printPolyCoeff();
		std::cout << "polyY: "; polyY.printPolyCoeff();
		std::cout << "polyZ: "; polyZ.printPolyCoeff();
}

void poly_3D::segmentAtTime(const double time,
	               			Eigen::Vector3d &result){
	polyX.polyAtTime(time, result[0]);
	polyY.polyAtTime(time, result[1]);
	polyZ.polyAtTime(time, result[2]);
}

void poly_3D::segmentAtTime(const double time,
	               			pcl::PointXYZ &result){
	double x, y, z;
	polyX.polyAtTime(time, x);
	polyY.polyAtTime(time, y);
	polyZ.polyAtTime(time, z);
	result.x = x; 
	result.y = y;
	result.z = z;
}

trajectory_3D::trajectory_3D(const astrobee_mapper::ControlGoal segments){
	nSegments = segments.segment.size() - 1;
	ros::Time t0_segment;
	ros::Time tf_segment;

	t0 = segments.segment[0].when.toSec();
	tf = segments.segment[nSegments].when.toSec();

    for(int i = 0; i < nSegments; i++){
    	t0_segment = segments.segment[i].when;
    	tf_segment = segments.segment[i+1].when;

    	poly_3D newSegment(t0_segment.toSec(),
    		               tf_segment.toSec(), 
    		               segments.segment[i]);
    	segments_poly.push_back(newSegment);
    }
}

void trajectory_3D::printTrajCoeff(){
	std::cout << "n_segments: " << nSegments << std::endl;
	std::cout << "t0: "   << t0
			  << "\ttf: " << tf << std::endl;
	for(int i = 0; i < nSegments; i++){
		std::cout << "Segment: " << i+1 << std::endl;
		segments_poly[i].printSegmentCoeff();
	}
}

void trajectory_3D::trajectoryAtTime(const double time,
	                      			 Eigen::Vector3d &result){
	
	//Find which segment the given "time" belongs to
	for(int i = 0; i < nSegments; i++){
		if(time <= segments_poly[i].tf){
			segments_poly[i].segmentAtTime(time,result);
			return;
		}
	}

	//If the code reaches this point, it means that we have to
	//return the value of the polynomial some time after tf (extrapolation)
	segments_poly[nSegments-1].segmentAtTime(time,result);
	ROS_WARN("Time extrapolation when returning polynomial!");
	return;
}

void trajectory_3D::trajectoryAtTime(const double time,
	                      			 pcl::PointXYZ &result){
	
	//Find which segment the given "time" belongs to
	for(int i = 0; i < nSegments; i++){
		if(time <= segments_poly[i].tf){
			segments_poly[i].segmentAtTime(time,result);
			return;
		}
	}

	//If the code reaches this point, it means that we have to
	//return the value of the polynomial some time after tf (extrapolation)
	segments_poly[nSegments-1].segmentAtTime(time,result);
	ROS_WARN("Time extrapolation when returning polynomial!");
	return;
}


