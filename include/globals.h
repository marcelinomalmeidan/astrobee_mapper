//ROS libraries
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_broadcaster.h>

//Astrobee message types
#include "ff_msgs/ControlGoal.h"
#include "ff_msgs/ControlState.h"

//My defined libraries
#include "tf_class.h"
#include "octoClass.h"
#include "structs.h"
#include "HelperFcns/visualizationFunctions.h"
#include "HelperFcns/QuatRotEuler.h"
#include "Services/services.h"
#include "threads/threads.h"
#include "polynomials.h"
#include "sampledTrajectory.h"
#include "Callbacks/callbacks.h" 

//Declare global variables (structures defined in structs.h)
extern mutexStruct mutexes;		
extern globalVariables globals;	
