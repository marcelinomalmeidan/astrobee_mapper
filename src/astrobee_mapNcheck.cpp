
#include "globals.h"

mutexStruct mutexes;		
globalVariables globals;


int main(int argc, char **argv) {

	ros::init(argc, argv, "mappingPkg");
	ros::NodeHandle n("~"); 

	//Load parameterss
	double mapResolution, memoryTime, maxRange, minRange, inflateRadius;
	double trajResolution, compressionMaxDev;
	bool pubTf, inflateMap;
	n.param("mapResolution", mapResolution, 0.1);
	n.param("maxRange", maxRange, 4.0);
	n.param("minRange", minRange, 0.2);
	n.param("memoryTime", memoryTime, 30.0);
	n.param("inflateRadius", inflateRadius, 0.25);
	n.param("inflateMap", inflateMap, true);
	n.param("pubTf", pubTf, false);
	n.param("trajResolution", trajResolution, 0.02);
	n.param("compressionMaxDev", compressionMaxDev, 0.01);

	//Update tree parameters
	globals.obsTree.setResolution(mapResolution);
	globals.obsTree.setMaxRange(maxRange);
	globals.obsTree.setMinRange(minRange);
	globals.obsTree.setMemory(memoryTime);
	globals.obsTree.setMapInflation(inflateMap, inflateRadius);

	//Update trajectory parameters
	globals.sampledTraj.setMaxDev(compressionMaxDev);
	globals.sampledTraj.setResolution(trajResolution);

	//Initialize mutexes
	initializeMutexes(mutexes);

	//Threads --------------------------------------------------
	pthread_t h_tfThread;      //tf listener thread
	pthread_t h_fadeThread;    //thread for fading memory of the map
	pthread_t h_tfPub;    	   //thread for fading memory of the map
	pthread_t h_collisionCheckThread; //Thread for collision checking
	int ReturnCode, threadCount;

	//Start  tf listener
	if (ReturnCode = pthread_create(&h_tfThread, NULL, tfTask, NULL)){
		printf("Start tf thread failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else{
		pthread_mutex_lock(&mutexes.threadCount);
		  globals.threadCount += 1;
		pthread_mutex_unlock(&mutexes.threadCount);
	}

	//Start fading memory thread
	if (ReturnCode = pthread_create(&h_fadeThread, NULL, fadeTask, NULL)){
		printf("Start fading memory thread failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else{
		pthread_mutex_lock(&mutexes.threadCount);
		  globals.threadCount += 1;
		pthread_mutex_unlock(&mutexes.threadCount);
	}

	//Start  collision check thread
	if (ReturnCode = pthread_create(&h_collisionCheckThread, NULL, collisionCheckTask, NULL)){
		printf("Start collisionCheck thread failed; return code from pthread_create() is %d\n", ReturnCode);
		exit(-1);
	}
	else{
		pthread_mutex_lock(&mutexes.threadCount);
		  globals.threadCount += 1;
		pthread_mutex_unlock(&mutexes.threadCount);
	}

	//Start tf publisher tree if requested
	if(pubTf){
		if (ReturnCode = pthread_create(&h_tfPub, NULL, tfPub, NULL)){
			printf("Start h_tfPub thread failed; return code from pthread_create() is %d\n", ReturnCode);
			exit(-1);
		}
		else{
			pthread_mutex_lock(&mutexes.threadCount);
			  globals.threadCount += 1;
			pthread_mutex_unlock(&mutexes.threadCount);
		}
	}

	//Create services ------------------------------------------
	ros::ServiceServer mapFreeNodes_srv = n.advertiseService("mapFreeNodes", mapFreeNodes);
	ros::ServiceServer Resolution_srv = n.advertiseService("updateResolution", updateResolution);
	ros::ServiceServer MemoryTime_srv = n.advertiseService("updateMemoryTime", updateMemoryTime);
	ros::ServiceServer mapInflation_srv = n.advertiseService("mapInflation", mapInflation);
	ros::ServiceServer ResetMap_srv = n.advertiseService("resetMap", resetMap);

	// Subscribers ----------------------------------------------
	ros::Subscriber hazSub = n.subscribe("/hw/depth_haz/points", 10, pclCallback);
	ros::Subscriber perchSub = n.subscribe("/hw/depth_perch/points", 10, pclCallback);
	ros::Subscriber segmentSub = n.subscribe("/gnc/ctl/segment", 10, segmentCallback);
	
	// Publishers -----------------------------------------------
	globals.obstacleMarker_pub = n.advertise<visualization_msgs::MarkerArray>("Obstacle_markers", 1000);
	globals.freeSpaceMarker_pub = n.advertise<visualization_msgs::MarkerArray>("FreeSpace_markers", 1000);
	globals.inflatedObstacleMarker_pub = n.advertise<visualization_msgs::MarkerArray>("InflatedObstacle_markers", 1000);
	globals.inflatedFreeSpaceMarker_pub = n.advertise<visualization_msgs::MarkerArray>("InflatedFreeSpace_markers", 1000);
	globals.pathMarker_pub = n.advertise<visualization_msgs::MarkerArray>("path_markers", 1000);

	ros::spin();

	//destroy mutexes
	destroyMutexes(mutexes);

	return 0;
}