#include "sampledTrajectory.h"

sampledTrajectory3D::sampledTrajectory3D(const double dt,
						                 trajectory_3D polyTrajectories){
	static double t0, tf, DeltaT;
	t0 = polyTrajectories.t0;
	tf = polyTrajectories.tf;
	DeltaT = tf - t0;

	//Define number of points for trajectory
	if(floor(DeltaT/dt) == DeltaT/dt){
		nPoints = DeltaT/dt + 1;
	}
	else{
		nPoints = DeltaT/dt + 2;
	}

	//Set vector values
	// Eigen::Vector3d Point;
	pcl::PointXYZ Point; 
	double time;
	for(int i = 0; i < nPoints; i++){
		time = std::min(float(i)*dt + t0,tf);
		Time.push_back(time);
		polyTrajectories.trajectoryAtTime(time, Point);
		Pos.push_back(Point);
	}
}

sampledTrajectory3D::sampledTrajectory3D(const std::vector<double> TimeVec,
		                				 const pcl::PointCloud<pcl::PointXYZ> PosVec){
	Time = TimeVec;
	Pos = PosVec;
	nPoints = Time.size();
}

sampledTrajectory3D::sampledTrajectory3D(){
	nPoints = 0;
	// Pos = std::vector<Eigen::Vector3d>;
}

void sampledTrajectory3D::printSamples(){
	for(int i = 0; i < nPoints; i++){
		std::cout << "Sample: " << i+1 << "\t";
		std::cout << "Time: " << Time[i] << "\t";
		std::cout << "Point: " << Pos[i].x << "\t" << Pos[i].y << "\t" << Pos[i].z << std::endl;
	}
}

void sampledTrajectory3D::setMaxDev(double maxDev_in){
	maxDev = maxDev_in;
	ROS_INFO("Trajectory compression max deviation is set to: %f", maxDev);
}

void sampledTrajectory3D::setResolution(double resolution_in){
	resolution = resolution_in;
	thickness = 1.4*resolution_in;
	ThickTraj.setResolution(resolution);
	ThickTraj.clear();
	ROS_INFO("Trajectory resolution is set to: %f", resolution);
}

void sampledTrajectory3D::deleteSample(const int index){
	compressedPos.erase(compressedPos.begin() + index);
	compressedTime.erase(compressedTime.begin() + index);
	compressedPoints = compressedPoints - 1;	
}

void sampledTrajectory3D::compressSamples(){
	//The minimum number of points for final vector
	static int minPoints = 2;

	//Initialize compressed points as all samples
	compressedPos.resize(nPoints);
	for (int i = 0; i < nPoints; i++){
		compressedPos[i] << Pos[i].x, Pos[i].y, Pos[i].z;
	}
	compressedTime = Time;
	compressedPoints = nPoints;

	//First delete colinear points
	static double epsilon = 0.0001, dist;
	static int deleteIndex;
	static Eigen::Vector3d p1, p2, p;
	static Line_3d line;
	while(true){
		int deleteIndex = -1;
		for(int i = 1; i < compressedPoints-1; i++){
			p1 << compressedPos[i-1][0], 
			      compressedPos[i-1][1], 
			      compressedPos[i-1][2];
			p2 << compressedPos[i+1][0], 
			      compressedPos[i+1][1], 
			      compressedPos[i+1][2];
			p  << compressedPos[i][0],   
			      compressedPos[i][1],   
			      compressedPos[i][2];
			this->lineThroughPoints(p1, p2, line);
			this->distancePoint2Line(p, line, dist);
			if(dist < epsilon){
				deleteIndex = i;
				break;
			}
		}
		if(deleteIndex > 0){
			this->deleteSample(deleteIndex);
		}
		else{
			break;
		}
	}
	// ROS_INFO("Number of non-colinear points: %d", compressedPoints);

	//Now compress the remaining points
	double minDist, initMinDist;
	while(compressedPoints > minPoints){
		//First find the point that deviates the least in the whole set
		minDist = std::numeric_limits<float>::infinity();
		for(int i = 1; i < compressedPoints-1; i++){
			p1 << compressedPos[i-1][0], 
			      compressedPos[i-1][1], 
			      compressedPos[i-1][2];
			p2 << compressedPos[i+1][0], 
			      compressedPos[i+1][1], 
			      compressedPos[i+1][2];
			p  << compressedPos[i][0],   
			      compressedPos[i][1],   
			      compressedPos[i][2];
			this->lineThroughPoints(p1, p2, line);
			this->distancePoint2Line(p, line, dist);
			if(dist < minDist){
				minDist = dist;
				deleteIndex = i;
			}
		}

		//If point does not deviate too much from original set, delete it
		if(minDist > maxDev){
			break;
		}
		else{
			this->deleteSample(deleteIndex);
		}
	}

	// ROS_INFO("Compressed points: %d", compressedPoints);

}


//It is highly likely that the original Author was Bob Pendelton
// I translated this algorithm from Matlab into C++ based on:
// http://www.mathworks.com/matlabcentral/fileexchange/21057-3d-bresenham-s-line-generation?focused=5102923&tab=function
void sampledTrajectory3D::bresenham(const Eigen::Vector3d p0, 
						      	    const Eigen::Vector3d pf,
						      	    std::vector<octomap::point3d> &Points){
	//Set initial and final pixel positions
	static int x1, x2, y1, y2, z1, z2;
	x1 = round(p0[0]/resolution);
	x2 = round(pf[0]/resolution);
	y1 = round(p0[1]/resolution);
	y2 = round(pf[1]/resolution);
	z1 = round(p0[2]/resolution);
	z2 = round(pf[2]/resolution);

	//Get the output vector length
	static int dx, dy, dz;
	dx = x2 - x1;
	dy = y2 - y1;
	dz = z2 - z1;
	const int d = std::max({abs(dx), abs(dy), abs(dz)}) + 1;
	// Eigen::MatrixXi Pixels(d,3);
	Points.reserve(d);

	//Extra variables
	static int ax, ay, az, sx, sy, sz, x, y, z, xd, yd, zd, idx;
	ax = abs(dx)*2;
	ay = abs(dy)*2;
	az = abs(dz)*2;
	sx = (dx > 0) - (dx < 0); //Sign of dx
	sy = (dy > 0) - (dy < 0); //Sign of dy
	sz = (dz > 0) - (dz < 0); //Sign of dz
	x = x1;
	y = y1;
	z = z1;
	// idx = 0;
 
	Eigen::MatrixXi Pixel(1,3);
	octomap::point3d Point;
	if(ax >= std::max(ay,az)){ //x dominant
		yd = ay - ax/2;
		zd = az - ax/2;

		while(true){
			// Pixel << x, y, z;
			// Pixels.row(idx) = Pixel;
			// idx = idx + 1;
			Point = octomap::point3d(double(x)*resolution, 
			         				 double(y)*resolution, 
			         				 double(z)*resolution);
			Points.push_back(Point); 

			if(x == x2){
				break;
			}

			if(yd >=0){ //Move along y
				y = y + sy;
				yd = yd - ax;
			}

			if(zd >= 0){ //Move along z
				z = z + sz;
				zd = zd - ax;
			}

			x = x + sx;	//Move along x
			yd = yd + ay;
			zd = zd + az;
		}
	}
	else if(ay >= std::max(ax,az)){	//y dominant
		xd = ax - ay/2;
		zd = az - ay/2;

		while(true){
			// Pixel << x, y, z;
			// Pixels.row(idx) = Pixel;
			// idx = idx + 1;
			Point = octomap::point3d(double(x)*resolution, 
			         				 double(y)*resolution, 
			         				 double(z)*resolution);
			Points.push_back(Point); 

			if(y == y2){ 
				break;
			}

			if(xd >= 0){
				x = x + sx;
				xd = xd - ay;
			}

			if(zd >= 0){
				z = z + sz;
				zd = zd - ay;
			}

			y = y + sy;
			xd = xd + ax;
			zd = zd + az;
		}
	}
	else if(az >= std::max(ax,ay)){	//z dominant
		xd = ax - az/2;
		yd = ay - az/2;

		while(true){
			// Pixel << x, y, z;
			// Pixels.row(idx) = Pixel;
			// idx = idx + 1;
			Point = octomap::point3d(double(x)*resolution, 
			         				 double(y)*resolution, 
			         				 double(z)*resolution);
			Points.push_back(Point); 

			if(z == z2){
				break;
			}

			if(xd >= 0){
				x = x + sx;
				xd = xd - az;
			}

			if(yd >=0){
				y = y + sy;
				yd = yd - az;
			}

			z = z + sz;
			xd = xd + ax;
			yd = yd + ay;
		}
	}

      
}

//Algorithm for getting a "thick" line based on bresenham
// TODO: Some voxels are painted twice, which can be improved.
void sampledTrajectory3D::thickBresenham(const Eigen::Vector3d p0, 
							      	     const Eigen::Vector3d pf){
	//V1: vector in the direction of the trajectory
	//V2 and V3: Orthogonal to V1. V2 and V3 are also orthogonal
	static Eigen::Vector3d V1, V2, V3;

	//Get vector in the direction of the trajectory (if length is greater than 0)
	V1 << pf - p0;
	if (V1.norm() > 0){
		V1 = V1/V1.norm();
	}
	else{ 
		V1 << 1, 0, 0;
	}

	//Find V2 and V3
	if(V1(0) != 0){
		V2(1) = 1;
		V2(2) = 1;
		V2(0) = -(V2(1)*V1(1) + V2(2)*V1(2))/V1(0);
		V2 = V2/V2.norm();
	}
	else if(V1(1) != 0){
		V2(0) = 1;
		V2(2) = 1;
		V2(1) = -(V2(0)*V1(0) + V2(2)*V1(2))/V1(1);
		V2 = V2/V2.norm();
	}
	else{
		V2(0) = 1;
		V2(1) = 1;
		V2(2) = -(V2(0)*V1(0) + V2(1)*V1(1))/V1(2);
		V2 = V2/V2.norm();
	}
	V3 = V1.cross(V2);


	//Get all pixels in a sphere around the origin
	//The pixels are assigned as initialEdge, finalEdge or trajectory padding
	std::vector<octomap::point3d> padding, initEdgePadding, finalEdgePadding;
	static octomap::point3d XYZf, V1_;
	const int maxXYZ = (int)round(thickness/resolution);
	const float maxD = thickness*thickness;
	static float d_origin, D_plane;
	V1_ = octomap::point3d(V1[0],V1[1],V1[2]);
	for(int x = -maxXYZ; x <= maxXYZ; x++){
		for(int y = -maxXYZ; y <= maxXYZ; y++){
			for(int z = -maxXYZ; z <= maxXYZ; z++){
				XYZf = octomap::point3d(x*resolution, y*resolution, z*resolution);

				d_origin = XYZf.dot(XYZf); //Distance from origin squared
				if(d_origin > maxD){
					continue;					
				}

				//Assign to proper vector
				D_plane = V1_.dot(XYZf);
				if(D_plane > 0.85*resolution){
					finalEdgePadding.push_back(XYZf);
				}
				else if(D_plane < -0.5*resolution){
					initEdgePadding.push_back(XYZf);
				}
				else{
					padding.push_back(XYZf);
				}
			}
		}
	}

	//Get the standard bresenham
	std::vector<octomap::point3d> thinBresenham;
	this->bresenham(p0, pf, thinBresenham);

	//Vector that is used to displace pixels based on origin at 0
	// to pixels in octree based origin (there is no pixel at the)
	// origin in octrees
	static octomap::point3d ds;
	ds = octomap::point3d(resolution/2.0, resolution/2.0, resolution/2.0);

	//Add padding around thinBresenham to get thickBresenham
	std::vector<Eigen::Vector3d> thickBresenham;
	const bool lazy_eval = true;
	for(int i = 0; i < thinBresenham.size(); i++){
		for(int j = 0; j < padding.size(); j++){
			// Pixels.push_back(thinBresenham[i] + padding[j] + ds);
			ThickTraj.updateNode(thinBresenham[i] + padding[j] + ds,true, lazy_eval);
		}
	}

	//Add edge padding
	for(int j = 0; j < initEdgePadding.size(); j++){
		// Pixels.push_back(thinBresenham[0] + initEdgePadding[j] + ds);
		ThickTraj.updateNode(thinBresenham[0] + initEdgePadding[j] + ds,true, lazy_eval);
	}
	for(int j = 0; j < finalEdgePadding.size(); j++){
		// Pixels.push_back(thinBresenham[thinBresenham.size()-1] + finalEdgePadding[j] + ds);
		ThickTraj.updateNode(thinBresenham[thinBresenham.size()-1] + finalEdgePadding[j] + ds,true, lazy_eval);
	}

	// std::cout << initEdgePadding.size() << std::endl;
	// Pixels = padding;

	// ROS_INFO("xmin = %d; xmax = %d; ymin = %d; ymax = %d; zmin = %d; zmax = %d;", xmin, xmax, ymin, ymax, zmin, zmax);

}

void sampledTrajectory3D::thickTrajToPcl(){

	pcl::PointXYZ Point;
	PointCloudTraj.clear();

	//Iterate through trajectory points
	for(octomap::OcTree::leaf_iterator it = ThickTraj.begin_leafs(), 
		                               end= ThickTraj.end_leafs(); 
		                               it!= end; ++it){
		Point.x = it.getX();
		Point.y = it.getY();
		Point.z = it.getZ();
		PointCloudTraj.push_back(Point);
	}

}

void sampledTrajectory3D::createKdTree(){
	*cloudPtr = Pos;
	kdtreePos.setInputCloud(cloudPtr);
}

void sampledTrajectory3D::sortCollisions(const std::vector<octomap::point3d> &collidingNodes,
	                                     std::vector<geometry_msgs::PointStamped> &Samples){
	pcl::PointXYZ searchPoint;
	int K = 1; //Search for 1 nearest neighbor
	std::vector<int> pointIdx(K);
	std::vector<float> pointSqrDist(K);
	geometry_msgs::PointStamped Sample;
	Samples.resize(collidingNodes.size());

	for(int i = 0; i < collidingNodes.size(); i++){
		searchPoint.x = collidingNodes[i].x();
		searchPoint.y = collidingNodes[i].y();
		searchPoint.z = collidingNodes[i].z();
		kdtreePos.nearestKSearch (searchPoint, K, pointIdx, pointSqrDist);
		Sample.point.x = cloudPtr->points[pointIdx[0]].x;
		Sample.point.y = cloudPtr->points[pointIdx[0]].y;
		Sample.point.z = cloudPtr->points[pointIdx[0]].z;
		Sample.header.stamp = ros::Time(Time[pointIdx[0]]);
		Samples[i] = Sample;
	}
	std::sort(Samples.begin(), Samples.end(), comparePointStamped);

}


void sampledTrajectory3D::trajVisMarkers(visualization_msgs::MarkerArray* marker_array){	//Publish occupied nodes
	// Markers: each marker array stores a set of nodes with similar size
	// visualization_msgs::MarkerArray occupiedNodesVis;
	const int treeDepth = ThickTraj.getTreeDepth();
	marker_array->markers.resize(treeDepth+1);
	const ros::Time rostime = ros::Time::now();

	//Set color parameters
	std_msgs::ColorRGBA color;
	color = Color::Orange();
	color.a = 0.15;

	//Publish all leafs from the tree
	for(octomap::OcTree::leaf_iterator it = ThickTraj.begin_leafs(), 
		                               end= ThickTraj.end_leafs(); 
		                               it!= end; ++it){
		if (ThickTraj.isNodeOccupied(*it)){
			//Get depth in the tree
			unsigned idx = it.getDepth();
			geometry_msgs::Point PointCenter;
			PointCenter.x = it.getX();
			PointCenter.y = it.getY();
			PointCenter.z = it.getZ();
			marker_array->markers[idx].points.push_back(PointCenter);

			//Set color based on height
			// double h = (1.0 - std::min(std::max((PointCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0))*colorFactor;
			marker_array->markers[idx].colors.push_back(color);
		}
	}

	//Set marker properties
    for (unsigned i= 0; i < marker_array->markers.size(); ++i){
		double size = ThickTraj.getNodeSize(i);

		marker_array->markers[i].header.frame_id = "world";
		marker_array->markers[i].header.stamp = rostime;
		marker_array->markers[i].ns = "thickTraj";
		marker_array->markers[i].id = i;
		marker_array->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
		marker_array->markers[i].scale.x = size;
		marker_array->markers[i].scale.y = size;
		marker_array->markers[i].scale.z = size;

		if (marker_array->markers[i].points.size() > 0)
			marker_array->markers[i].action = visualization_msgs::Marker::ADD;
		else
			marker_array->markers[i].action = visualization_msgs::Marker::DELETE;
    }
}

void sampledTrajectory3D::samplesVisMarkers(visualization_msgs::MarkerArray* marker_array){
	marker_array->markers.resize(1);
	const ros::Time rostime = ros::Time::now();

	//Set color parameters
	std_msgs::ColorRGBA color;
	color = Color::Blue();
	color.a = 0.5;

	//Publish all leafs from the tree
	marker_array->markers[0].points.resize(Pos.size());
	marker_array->markers[0].colors.resize(Pos.size());
	for(int i = 0; i < Pos.size(); i++){
		geometry_msgs::Point Point;
		Point.x = Pos[i].x;
		Point.y = Pos[i].y;
		Point.z = Pos[i].z;
		marker_array->markers[0].points.push_back(Point);
		marker_array->markers[0].colors.push_back(color);
	}

	//Set marker properties
	marker_array->markers[0].header.frame_id = "world";
	marker_array->markers[0].header.stamp = rostime;
	marker_array->markers[0].ns = "SampledTraj";
	marker_array->markers[0].id = 0;
	marker_array->markers[0].type = visualization_msgs::Marker::CUBE_LIST;
	marker_array->markers[0].scale.x = resolution;
	marker_array->markers[0].scale.y = resolution;
	marker_array->markers[0].scale.z = resolution;

	if (marker_array->markers[0].points.size() > 0)
		marker_array->markers[0].action = visualization_msgs::Marker::ADD;
	else
		marker_array->markers[0].action = visualization_msgs::Marker::DELETE;
}

void sampledTrajectory3D::compressedVisMarkers(visualization_msgs::MarkerArray* marker_array){
	marker_array->markers.resize(1);
	const ros::Time rostime = ros::Time::now();

	//Set color parameters
	std_msgs::ColorRGBA color;
	color = Color::Green();
	color.a = 0.9;

	//Publish all leafs from the tree
	marker_array->markers[0].points.resize(Pos.size());
	marker_array->markers[0].colors.resize(Pos.size());
	for(int i = 0; i < compressedPos.size(); i++){
		geometry_msgs::Point Point;
		Point.x = compressedPos[i][0];
		Point.y = compressedPos[i][1];
		Point.z = compressedPos[i][2];
		marker_array->markers[0].points.push_back(Point);
		marker_array->markers[0].colors.push_back(color);
	}

	//Set marker properties
	marker_array->markers[0].header.frame_id = "world";
	marker_array->markers[0].header.stamp = rostime;
	marker_array->markers[0].ns = "compressedTraj";
	marker_array->markers[0].id = 0;
	marker_array->markers[0].type = visualization_msgs::Marker::CUBE_LIST;
	marker_array->markers[0].scale.x = resolution;
	marker_array->markers[0].scale.y = resolution;
	marker_array->markers[0].scale.z = resolution;

	if (marker_array->markers[0].points.size() > 0)
		marker_array->markers[0].action = visualization_msgs::Marker::ADD;
	else
		marker_array->markers[0].action = visualization_msgs::Marker::DELETE;
}


//Private methods -----------------------------------------------

//3d_Line line that goes through two points p1, p2 in 2D
void sampledTrajectory3D::lineThroughPoints(const Eigen::Vector3d p1,
									        const Eigen::Vector3d p2,
									        Line_3d &line){
	// Line_3d line;
	line.p0 = p1;
	line.vec = p1 - p2;

}

//Calculate the distance between one point and a line in 2D
void sampledTrajectory3D::distancePoint2Line(const Eigen::Vector3d point,
				          					 const Line_3d line,
				          					 double &dist){

	//Two points on the line
	const Eigen::Vector3d x1 = line.p0;
	const Eigen::Vector3d x2 = line.p0 + line.vec;

	//Optimal t is the t at which the point is closest to the line
	// t_opt = (x1-x2)'*(x1-p)/norm(x1-x2)^2;
	static double t_opt;
	if(x1 == x2){
		t_opt = 0;
	}
	else{
		const double gain = 1.0/(pow((x1-x2).norm(),2.0));
		t_opt = gain*(x1-x2).transpose()*(x1-point);
	}

	//p_opt is the closest point between the point and the line
	const Eigen::Vector3d p_opt = x1 + t_opt*line.vec;

	dist = (point - p_opt).norm();
}

//Return the sample with lowest time
bool comparePointStamped(const geometry_msgs::PointStamped sample1, 
                         const geometry_msgs::PointStamped sample2){
	return sample1.header.stamp.toSec() < sample2.header.stamp.toSec();
}