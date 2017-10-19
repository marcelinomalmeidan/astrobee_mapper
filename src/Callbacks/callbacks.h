#ifndef _H_CALLBACKS_
#define _H_CALLBACKS_

#include "globals.h"

//Callback for handling incoming point cloud messages
void pclCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

//Callback for handling incoming new trajectory messages
void segmentCallback(const astrobee_mapper::ControlGoal::ConstPtr &msg);

#endif