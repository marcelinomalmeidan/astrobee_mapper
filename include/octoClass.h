#ifndef _H_OCTOMAP_CLASS_
#define _H_OCTOMAP_CLASS_


#include <iostream>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <visualization_msgs/MarkerArray.h>
#include "HelperFcns/visualizationFunctions.h"

//3D occupancy grid
class OctoClass{
public:
	octomap::OcTree tree = octomap::OcTree(0.1);  // create empty tree with resolution 0.1
	octomap::OcTree treeInflated = octomap::OcTree(0.1);  // create empty tree with resolution 0.1
	float logUpdateVal = 0.2;
	int treeDepth;
	double resolution;
	double memoryTime;	//Fading memory of the tree in seconds
	double maxRange, minRange;
	bool mapFreeNodes; 	//Whether or not to map free nodes (more expensive computationally)
	bool inflateMap;
	float inflateRadius;

	//Constructor
	OctoClass(const double resolution_in); //Resolution in meters
	OctoClass(); 

	//Methods
	void setMemory(const double memory);
	void setMaxRange(const double maxRange_in);
	void setMinRange(const double minRange_in);
	void setResolution(const double resolution_in);
	void resetMap();
	void setMapFreeNode(const bool mapFreeNodes_in);
	void setMapInflation(const bool inflateMap_in, 
		                 const double inflateRadius_in);
	void print_query_info(octomap::point3d query, 
		                  octomap::OcTreeNode* node);
	void pointsOctomapToPointCloud2(const octomap::point3d_list& points, 
		                            sensor_msgs::PointCloud2& cloud);
	void pointCloud2ToOctomap(const sensor_msgs::PointCloud2& cloud,
							  const tf::StampedTransform &tf_cam2world);	//Map only obstacles
	void pointCloud2ToRayOctomap(const sensor_msgs::PointCloud2& cloud,
								 const tf::StampedTransform &tf_cam2world);	//Map obstacles and free area
	void fadeMemory(const double &rate);	//Rate at which this function is being called
	void inflateObstacles(const double &thickness);
	void occupiedVisMarkers(visualization_msgs::MarkerArray* marker_array);
	void inflatedOccVisMarkers(visualization_msgs::MarkerArray* marker_array);
	void freeVisMarkers(visualization_msgs::MarkerArray* marker_array);
	void inflatedFreeVisMarkers(visualization_msgs::MarkerArray* marker_array);

private:
	std::vector<Eigen::Vector3d> sphere;

	//Methods
	double VectorNormSquared(const double &x, 
	                         const double &y, 
	                         const double &z);
	std_msgs::ColorRGBA heightMapColor(double h, double alpha);
};


#endif