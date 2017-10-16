#ifndef _H_STRUCTS_
#define _H_STRUCTS_

#include <pthread.h>

// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "octoClass.h"




struct globalVariables{
	//Mutex protected variables
	int threadCount;
	tf::StampedTransform tf_cam2world;
	tf::StampedTransform tf_perch2world;
	OctoClass obsTree = OctoClass(0.05);

	//publishers
	ros::Publisher obstacleMarker_pub;
	ros::Publisher freeSpaceMarker_pub;
	ros::Publisher inflatedObstacleMarker_pub;

	
};

struct mutexStruct{
	pthread_mutex_t threadCount;
	pthread_mutex_t segments;
	pthread_mutex_t tf;
	pthread_mutex_t obsTree;
};

void initializeMutexes(mutexStruct &mutexes);

void destroyMutexes(mutexStruct &mutexes);

#endif