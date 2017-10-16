#ifndef _H_SERVICES_
#define _H_SERVICES_

#include "globals.h"
#include "astrobee_mapper/updateMapFloatParam.h"
#include "astrobee_mapper/updateMapBoolParam.h"
#include "astrobee_mapper/updateMapInflation.h"
#include "astrobee_mapper/voidService.h"

//Update resolution of the map
bool updateResolution(astrobee_mapper::updateMapFloatParam::Request &req,
	                  astrobee_mapper::updateMapFloatParam::Response &res);

//Update map memory time
bool updateMemoryTime(astrobee_mapper::updateMapFloatParam::Request &req,
	                  astrobee_mapper::updateMapFloatParam::Response &res);

//Update whether or not to map free nodes
bool mapFreeNodes(astrobee_mapper::updateMapBoolParam::Request &req,
	              astrobee_mapper::updateMapBoolParam::Response &res);

//Update map inflation
bool mapInflation(astrobee_mapper::updateMapInflation::Request &req,
	              astrobee_mapper::updateMapInflation::Response &res);

//Reset the map
bool resetMap(astrobee_mapper::voidService::Request &req,
	          astrobee_mapper::voidService::Response &res);


#endif