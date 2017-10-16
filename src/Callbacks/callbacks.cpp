
#include "callbacks.h"

void pclCallback(const sensor_msgs::PointCloud2::ConstPtr &msg){

	//Get time for when this task started
	const ros::Time t0 = ros::Time::now();

    //Get tf to transform pcl into world frame
    tf::StampedTransform tf_cam2world;
	pthread_mutex_lock(&mutexes.tf);
		if(!msg->header.frame_id.compare("haz_cam")){
			tf_cam2world = globals.tf_cam2world;
		}
		else if(!msg->header.frame_id.compare("perch_cam")){
			tf_cam2world = globals.tf_perch2world;
		}
	pthread_mutex_unlock(&mutexes.tf);

	//Check if a tf message has been received already. If not, return
	if(tf_cam2world.stamp_.toSec() == 0){
		return;
	}

	//Transform pcl into world frame
    sensor_msgs::PointCloud2 pcl;
    const std::string target_frame = "/world";
    pcl_ros::transformPointCloud(target_frame,tf_cam2world,*msg,pcl);


	//Save into octomap
	pthread_mutex_lock(&mutexes.obsTree);
		if(globals.obsTree.mapFreeNodes){
			globals.obsTree.pointCloud2ToRayOctomap(pcl, tf_cam2world);
		}
		else{
			globals.obsTree.pointCloud2ToOctomap(pcl, tf_cam2world);
		}
		globals.obsTree.tree.prune();	//Prune the tree before visualizing
		// globals.obsTree.tree.writeBinary("simple_tree.bt");
	pthread_mutex_unlock(&mutexes.obsTree);


	//Publish visualization markers iff at least one node is subscribed to it
	if(globals.obstacleMarker_pub.getNumSubscribers() > 0){
		visualization_msgs::MarkerArray obstacleMarkers;
		pthread_mutex_lock(&mutexes.obsTree);
			globals.obsTree.occupiedVisMarkers(&obstacleMarkers);
		pthread_mutex_unlock(&mutexes.obsTree);
		globals.obstacleMarker_pub.publish(obstacleMarkers);
	}

	if(globals.freeSpaceMarker_pub.getNumSubscribers() > 0){
		visualization_msgs::MarkerArray freeMarkers;
		pthread_mutex_lock(&mutexes.obsTree);
			globals.obsTree.freeVisMarkers(&freeMarkers);
		pthread_mutex_unlock(&mutexes.obsTree);
		globals.freeSpaceMarker_pub.publish(freeMarkers);
	}

	if(globals.inflatedObstacleMarker_pub.getNumSubscribers() > 0){
		visualization_msgs::MarkerArray inflatedMarkers;
		pthread_mutex_lock(&mutexes.obsTree);
			globals.obsTree.inflatedOccVisMarkers(&inflatedMarkers);
		pthread_mutex_unlock(&mutexes.obsTree);
		globals.inflatedObstacleMarker_pub.publish(inflatedMarkers);
	}
	

	ros::Duration mapTime = ros::Time::now() - t0;
	ROS_INFO("Mapping time: %f", mapTime.toSec());

}