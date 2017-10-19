 #ifndef _H_SAMPLE_TRAJ_CLASS_
#define _H_SAMPLE_TRAJ_CLASS_

#include <Eigen/Dense>
#include <iostream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "polynomials.h"
#include "HelperFcns/visualizationFunctions.h"

//Line parameterized as l = p0 + t.vec, t belongs to (-inf,inf)
struct Line_3d{
	Eigen::Vector3d p0;
	Eigen::Vector3d vec;
};

//  Pos has the discretized points in the trajectory,
//and is compressed with the function compressSamples
//  Time has the corresponding times within Pos
//  nPoints has the number of points in the Pos vector
//  ThickTraj is of the octree type to avoid repeated nodes
//that occurs when concatenating trajectories between two
//waypoints
class sampledTrajectory3D{
public:
	//Sampled trajectory variables
	pcl::PointCloud<pcl::PointXYZ> Pos;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtreePos;
	std::vector<double> Time;
	int nPoints;

	//Compressed samples
	std::vector<Eigen::Vector3d> compressedPos;
	std::vector<double> compressedTime;
	int compressedPoints;	//Number of points after compression
	double maxDev;			//Max deviation for compression

	//Thick trajectory variables
	octomap::OcTree ThickTraj = octomap::OcTree(0.1);  // create empty tree with resolution 0.1
	pcl::PointCloud< pcl::PointXYZ > PointCloudTraj;
	double resolution;
	double thickness;


	//Constructor
	sampledTrajectory3D(const double dt,
		                const trajectory_3D polyTrajectories);
	sampledTrajectory3D(const std::vector<double> TimeVec,
		                const pcl::PointCloud<pcl::PointXYZ> PosVec);
	sampledTrajectory3D(); 

	//Methods
	void printSamples();
	void setMaxDev(double maxDev_in);
	void setResolution(double resolution_in);
	void deleteSample(const int index);
	void compressSamples();
	void bresenham(const Eigen::Vector3d p0, 
		      	   const Eigen::Vector3d pf,
		      	   std::vector<octomap::point3d> &Pixels); //Bresenham line algorithm por printing a line
	void thickBresenham(const Eigen::Vector3d p0, 
		        	    const Eigen::Vector3d pf); //Bresenham line algorithm por printing a line
	void thickTrajToPcl();
	void createKdTree();
	void sortCollisions(const std::vector<octomap::point3d> &collidingNodes,
	                    std::vector<geometry_msgs::PointStamped> &Samples);
	void trajVisMarkers(visualization_msgs::MarkerArray* marker_array);
	void samplesVisMarkers(visualization_msgs::MarkerArray* marker_array);
	void compressedVisMarkers(visualization_msgs::MarkerArray* marker_array);

private:

	//Find line that goes through two points p1, p2
	void lineThroughPoints(const Eigen::Vector3d p1,
					       const Eigen::Vector3d p2,
					       Line_3d &line);

	//Calculate the distance between one point and a line
	void distancePoint2Line(const Eigen::Vector3d point,
					    	const Line_3d line,
					        double &dist);

};

//Comparison funcion used in sort algorithm
bool comparePointStamped(const geometry_msgs::PointStamped sample1, 
                         const geometry_msgs::PointStamped sample2); 


#endif