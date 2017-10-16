
#include "threads.h"

//Thread for fading memory of the octomap
void *fadeTask(void *threadID){
	ROS_DEBUG("Fading Memory Thread started!");

	//Rate at which this thread will run
	double rate = 1.0; //Rate in Hz
	ros::Rate loop_rate(rate);

	while (ros::ok()){

		//Get time for when this task started
		const ros::Time t0 = ros::Time::now();

		pthread_mutex_lock(&mutexes.obsTree);
			if(globals.obsTree.memoryTime > 0){
				globals.obsTree.fadeMemory(rate);
			}
		pthread_mutex_unlock(&mutexes.obsTree);

		ros::Duration fadeTime = ros::Time::now() - t0;
		ROS_DEBUG("Fading memory execution time: %f", fadeTime.toSec());

		loop_rate.sleep();
	}

	ROS_DEBUG("Exiting Fading Memory Thread...");

	pthread_mutex_lock(&mutexes.threadCount);
        globals.threadCount -= 1;
    pthread_mutex_unlock(&mutexes.threadCount);
	pthread_exit(NULL);
}

//Thread for constantly updating the tfTree values
void *tfTask(void *threadID){
	ROS_DEBUG("tf Thread started!");
	tf_class obj_cam2world;
	tf_class obj_perch2world;
	// tf_class obj_rviz2world;

	//Rate at which this thread will run
	ros::Rate loop_rate(10);


	while (ros::ok()){

		//Get the transforms
		obj_cam2world.getTransform("/haz_cam","/world");
		obj_perch2world.getTransform("/perch_cam","/world");
		// obj_rviz2world.getTransform("/rviz","/world");

		pthread_mutex_lock(&mutexes.tf);
			globals.tf_cam2world = obj_cam2world.transform;
			globals.tf_perch2world = obj_perch2world.transform;
		pthread_mutex_unlock(&mutexes.tf);


		// obj_rviz2world.printTransform();

		loop_rate.sleep();
	}

	ROS_DEBUG("Exiting tf Thread...");

	pthread_mutex_lock(&mutexes.threadCount);
        globals.threadCount -= 1;
    pthread_mutex_unlock(&mutexes.threadCount);
	pthread_exit(NULL);
}

//Thread for publishing tfTree (used when loading real data without tf data)
void *tfPub(void *threadID){
	static tf::TransformBroadcaster br;
	ros::Rate loop_rate(10.0);

	tf::Transform transforRviz2World;transforRviz2World.setOrigin(tf::Vector3(0,0,0));
	transforRviz2World.setRotation(tf::Quaternion(1,0,0,0));

	tf::Transform t_imu2haz, t_imu2nav, t_body2imu, t_imu2perch;
	geometry_msgs::Quaternion q_imu2haz, q_imu2nav, q_body2imu;
	Eigen::Matrix3d R_imu2haz, R_imu2nav;
	t_imu2haz.setOrigin(tf::Vector3(0.01982814, -0.10260928, -0.0809614));
	t_imu2nav.setOrigin(tf::Vector3(-0.05811117,-0.10935581,-0.08083976));
	t_body2imu.setOrigin(tf::Vector3(0.014, 0.016, -0.0319625));
	R_imu2nav << 0.9998663720981906, 0.006173216644918429, -0.01513701897527766,
				 0.015147935861241673, -0.0017255195197513618, 0.9998837745565886,
				 0.006146379938363911, -0.999979456778309, -0.0018188004319931728;
	R_imu2haz << -0.99999509, -0.00307496,  0.00060345,
	             -0.00065182,  0.01575341, -0.99987569,
	              0.00306507, -0.99987118, -0.01575533;
	q_imu2nav = rot2quat(R_imu2nav.transpose());
	q_imu2haz = rot2quat(R_imu2haz);
	q_body2imu = setQuat(0.0043601096, -0.0066617904, 0.75077957, 0.66050535);
	t_imu2haz.setRotation(tf::Quaternion(q_imu2haz.x, q_imu2haz.y, q_imu2haz.z, q_imu2haz.w));
	t_imu2nav.setRotation(tf::Quaternion(q_imu2nav.x,q_imu2nav.y,q_imu2nav.z,q_imu2nav.w));
	t_body2imu.setRotation(tf::Quaternion(q_body2imu.x,q_body2imu.y,q_body2imu.z,q_body2imu.w));

	t_imu2perch.setOrigin(tf::Vector3(0,0,0));
	t_imu2perch.setRotation(tf::Quaternion(0,0,0,1));

	while (ros::ok()){

		br.sendTransform(tf::StampedTransform(transforRviz2World, ros::Time::now(), "rviz", "world"));
		br.sendTransform(tf::StampedTransform(t_imu2haz, ros::Time::now(), "imu", "haz_cam"));
		br.sendTransform(tf::StampedTransform(t_imu2nav, ros::Time::now(), "imu", "nav_cam"));
		br.sendTransform(tf::StampedTransform(t_imu2perch, ros::Time::now(), "imu", "perch_cam"));
		br.sendTransform(tf::StampedTransform(t_body2imu, ros::Time::now(), "body", "imu"));
		
		loop_rate.sleep();
	}
}