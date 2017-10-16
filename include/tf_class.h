#ifndef _H_TF_CLASS_
#define _H_TF_CLASS_

#include <tf/transform_listener.h>

class tf_class{
public:
	tf::TransformListener listener;
	tf::StampedTransform transform;

	//Constructor
	tf_class();

	//Methods
	void printTransform();
	bool getTransform(const std::string &original_frame,
	              	  const std::string &target_frame);

};


#endif