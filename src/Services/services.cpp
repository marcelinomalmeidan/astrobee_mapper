
#include "services.h"


//Update resolution of the map
bool updateResolution(astrobee_mapper::updateMapFloatParam::Request &req,
	                  astrobee_mapper::updateMapFloatParam::Response &res){
	
	pthread_mutex_lock(&mutexes.obsTree);
		globals.obsTree.setResolution(req.data);
	pthread_mutex_unlock(&mutexes.obsTree);	

	res.success = true;
	return true;
}

//Update map memory time
bool updateMemoryTime(astrobee_mapper::updateMapFloatParam::Request &req,
	                  astrobee_mapper::updateMapFloatParam::Response &res){
	
	pthread_mutex_lock(&mutexes.obsTree);
		globals.obsTree.setMemory(req.data);
	pthread_mutex_unlock(&mutexes.obsTree);	

	res.success = true;
	return true;
}

//Update whether or not to map free nodes
bool mapFreeNodes(astrobee_mapper::updateMapBoolParam::Request &req,
	              astrobee_mapper::updateMapBoolParam::Response &res){
	pthread_mutex_lock(&mutexes.obsTree);
		globals.obsTree.setMapFreeNode(req.data);
	pthread_mutex_unlock(&mutexes.obsTree);	

	// if(req.data){
	// 	ROS_INFO("Mapper is updating free nodes!");
	// }
	// else{
	// 	ROS_INFO("Mapper is not updating free nodes!");
	// }

	res.success = true;
	return true;
}

bool mapInflation(astrobee_mapper::updateMapInflation::Request &req,
	              astrobee_mapper::updateMapInflation::Response &res){
	pthread_mutex_lock(&mutexes.obsTree);
		globals.obsTree.setMapInflation(req.inflateMap, req.inflateRadius);
	pthread_mutex_unlock(&mutexes.obsTree);	

	res.success = true;
	return true;
}

bool resetMap(astrobee_mapper::voidService::Request &req,
	          astrobee_mapper::voidService::Response &res){
	pthread_mutex_lock(&mutexes.obsTree);
		globals.obsTree.resetMap();
	pthread_mutex_unlock(&mutexes.obsTree);	

	res.success = true;
	return true;
}