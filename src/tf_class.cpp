
#include "tf_class.h"

//Constructor
tf_class::tf_class(){

}

//Print everything within transform (for debugging)
void tf_class::printTransform(){
	tf::Quaternion q = transform.getRotation();
 	tf::Vector3 v = transform.getOrigin();
	double yaw, pitch, roll;
	transform.getBasis().getRPY(roll, pitch, yaw);
 	std::cout << "- Translation: [" << v.getX() << ", " 
 									<< v.getY() << ", " 
 									<< v.getZ() << "]" << std::endl;
 	std::cout << "- Rotation: in Quaternion [" 
 			  << q.getX() << ", " 
 			  << q.getY() << ", " 
    	      << q.getZ() << ", " 
    	      << q.getW() << "]" << std::endl
        	  << "            in RPY (radian) [" 
        	  <<  roll << ", " 
        	  << pitch << ", " 
        	  << yaw << "]" << std::endl
        	  << "            in RPY (degree) [" 
        	  <<  roll*180.0/M_PI << ", " 
        	  << pitch*180.0/M_PI << ", " 
        	  << yaw*180.0/M_PI << "]" << std::endl;
}

//Get transform from original to target frame
bool tf_class::getTransform(const std::string &original_frame,
              	  const std::string &target_frame){
	try{
		//First we wait until transform is published, then we look it up
		if(listener.waitForTransform(target_frame,original_frame,
			                         ros::Time(0), ros::Duration(2.0))){
			listener.lookupTransform(target_frame, original_frame,  
	                                 ros::Time(0), transform);
			}
		else{
			ROS_WARN("Transform not being published!");
			return false;
			}
		}
	catch (tf::TransformException &ex) {
	      ROS_WARN("%s",ex.what());
	      return false;
	    }

	return true;
}