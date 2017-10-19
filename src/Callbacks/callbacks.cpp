
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


void segmentCallback(const astrobee_mapper::ControlGoal::ConstPtr &msg){
	
	ros::Time t0 = ros::Time::now();
	while(t0.toSec() == 0){
		t0 = ros::Time::now();
	}
 
 	//Get segments
    astrobee_mapper::ControlGoal segments = *msg;

	//Transform message into set of polynomials
    trajectory_3D polyTrajectories(segments);

    //Get a fake trajectory
	std::vector<double> Time;
	pcl::PointCloud<pcl::PointXYZ> Positions; 
	pcl::PointXYZ Pos;
	double rx = 3, ry = 2, rz = 0;
	for (int i = 0; i < 100; i++){
		double time = 2*M_PI*float(i)/(100.0*4.0);
		Time.push_back(time);
		Pos = pcl::PointXYZ(rx*sin(time), ry*(cos(time)-1.0), rz*sin(time));
		Positions.push_back(Pos);
	}

    //Sample trajectory at 10hz
	sampledTrajectory3D sampledTraj(0.1, polyTrajectories);
	// sampledTrajectory3D sampledTraj(Time, Positions); //Fake trajectory
	
	pthread_mutex_lock(&mutexes.sampledTraj);
		globals.sampledTraj.Pos = sampledTraj.Pos;
		globals.sampledTraj.Time = sampledTraj.Time;
		globals.sampledTraj.nPoints = sampledTraj.nPoints;

		//Compress trajectory into points with max deviation of 1cm from original trajectory
		globals.sampledTraj.compressSamples();

		//  Transform compressed trajectory into a set of pixels in octomap
		//  Octomap insertion avoids repeated points
		globals.sampledTraj.ThickTraj.clear();
		for(int i = 0; i < globals.sampledTraj.compressedPoints-1; i++){
			globals.sampledTraj.thickBresenham(globals.sampledTraj.compressedPos[i], 
								       		   globals.sampledTraj.compressedPos[i+1]); 
		}

		//Populate trajectory node centers in a point cloud
		globals.sampledTraj.thickTrajToPcl();

		//Populate kdtree for finding nearest neighbor w.r.t. collisions
		globals.sampledTraj.createKdTree();
	pthread_mutex_unlock(&mutexes.sampledTraj);	

	ros::Duration SolverTime = ros::Time::now() - t0;
	ROS_INFO("Time to compute octotraj: %f", SolverTime.toSec());

} 