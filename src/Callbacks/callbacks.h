#ifndef _H_CALLBACKS_
#define _H_CALLBACKS_

#include "globals.h"

//Thread for handling incoming point cloud messages
void pclCallback(const sensor_msgs::PointCloud2::ConstPtr &msg);

#endif