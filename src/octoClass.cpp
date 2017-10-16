
#include "octoClass.h"

OctoClass::OctoClass(const double resolution_in){
	tree.setResolution(resolution_in);
	tree.setOccupancyThres(0.5);
	tree.setProbHit(0.6);
	tree.setProbMiss(0.4);
	tree.setClampingThresMin(0.1);
	tree.setClampingThresMax(0.9);
	treeDepth = tree.getTreeDepth();
	treeInflated.setResolution(resolution_in);
	resolution = resolution_in;
	memoryTime = 30.0;	//Fading memory of the tree in seconds
	mapFreeNodes = true;
	maxRange = 10.0;
	// tree.setResolution(resolution_in);
}

OctoClass::OctoClass(){
//Do nothing
}

void OctoClass::setMemory(const double memory){
	memoryTime = memory;
	ROS_INFO("Fading memory time: %f seconds", memoryTime);
}

void OctoClass::setMaxRange(const double maxRange_in){
	maxRange = maxRange_in;
	ROS_INFO("Maximum range: %f meters", maxRange);
}

void OctoClass::setMinRange(const double minRange_in){
	minRange = minRange_in;
	ROS_INFO("Minimum range: %f meters", minRange);
}

void OctoClass::setResolution(const double resolution_in){
	resolution = resolution_in;
	tree.setResolution(resolution);
	treeInflated.setResolution(resolution);
	this->resetMap();
	ROS_INFO("Map resolution: %f meters", resolution);
}

void OctoClass::setMapFreeNode(const bool mapFreeNodes_in){
	mapFreeNodes = mapFreeNodes_in;

	if(mapFreeNodes_in){
		ROS_INFO("Mapper is updating free nodes!");
	}
	else{
		ROS_INFO("Mapper is not updating free nodes!");
	}
}

void OctoClass::setMapInflation(const bool inflateMap_in, 
		                       const double inflateRadius_in){
	inflateMap = inflateMap_in;
	inflateRadius = inflateRadius_in;

	this->resetMap();

	sphere.clear();
	static Eigen::Vector3d XYZf;
	if(inflateMap){
		ROS_INFO("The map is being inflated by a radius of %f!", inflateRadius);
		const int maxXYZ = (int)round(inflateRadius/resolution);
		const float maxD = inflateRadius*inflateRadius;
		static float d_origin, D_plane;
		for(int x = -maxXYZ; x <= maxXYZ; x++){
			for(int y = -maxXYZ; y <= maxXYZ; y++){
				for(int z = -maxXYZ; z <= maxXYZ; z++){
					XYZf << x*resolution, y*resolution, z*resolution;

					d_origin = XYZf.dot(XYZf); //Distance from origin squared
					if(d_origin <= maxD){
						sphere.push_back(XYZf);					
					}

				}
			}
		}
	}
	else{
		ROS_INFO("The map is not currently being inflated!");
		XYZf << 0.0, 0.0, 0.0;
		sphere.push_back(XYZf);
	}

}

void OctoClass::resetMap(){
	tree.clear();
	treeInflated.clear();
	ROS_INFO("Map was reset!");
}
 
void OctoClass::print_query_info(octomap::point3d query, 
		                        octomap::OcTreeNode* node) {
	if (node != NULL) {
		std::cout << "occupancy probability at " << query << ":\t " << node->getOccupancy() << std::endl;
	}
	else{
		std::cout << "occupancy probability at " << query << ":\t is unknown" << std::endl;    
	}
}

//Function obtained from https://github.com/OctoMap/octomap_ros
void OctoClass::pointsOctomapToPointCloud2(const octomap::point3d_list& points, 
	                                      sensor_msgs::PointCloud2& cloud){
	// make sure the channel is valid
    std::vector<sensor_msgs::PointField>::const_iterator field_iter = cloud.fields.begin(), field_end =
        cloud.fields.end();
    bool has_x, has_y, has_z;
    has_x = has_y = has_z = false;
    while (field_iter != field_end) {
		if ((field_iter->name == "x") || (field_iter->name == "X"))
			has_x = true;
		if ((field_iter->name == "y") || (field_iter->name == "Y"))
			has_y = true;
		if ((field_iter->name == "z") || (field_iter->name == "Z"))
			has_z = true;
		++field_iter;
    }

    if ((!has_x) || (!has_y) || (!has_z))
		throw std::runtime_error("One of the fields xyz does not exist");

    sensor_msgs::PointCloud2Modifier pcd_modifier(cloud);
    pcd_modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for (octomap::point3d_list::const_iterator it = points.begin(); it != points.end(); ++it, ++iter_x, ++iter_y, ++iter_z) {
		*iter_x = it->x();
		*iter_y = it->y();
		*iter_z = it->z();
    }
}

//Function adapted from https://github.com/OctoMap/octomap_ros
void OctoClass::pointCloud2ToOctomap(const sensor_msgs::PointCloud2& cloud,
								    const tf::StampedTransform &tf_cam2world){
	// octomapCloud.reserve(cloud.data.size() / cloud.point_step);

	sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

	//Get camera origin
	const tf::Vector3 v = tf_cam2world.getOrigin();
	const double minThresholdSquare = minRange*minRange;
	const double maxThresholdSquare = maxRange*maxRange;
	static double rangeSqr;

	static octomap::point3d occPoint;
	// static octomap::OcTreeNode *node;
	// static bool isOcc, isOcc2;
	// static int counter;
	// counter = 0;
	for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
		//Points too close to origin of camera are not added
		rangeSqr = this->VectorNormSquared(v.getX()-*iter_x, v.getY()-*iter_y,v.getZ()-*iter_z);
		if((rangeSqr < minThresholdSquare) || (rangeSqr > maxThresholdSquare)){
			continue;
		}

		// Check if the point is invalid
		if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z)){

			octomap::OcTreeNode *node = tree.search(*iter_x, *iter_y, *iter_z);
			
			if(node == NULL){	//Create new node if non-existent
				// isOcc = false;
				occPoint = octomap::point3d(*iter_x, *iter_y, *iter_z);
				node = tree.updateNode(occPoint, logUpdateVal); //Update logval
				// counter = counter + 1;
			}
			else{	//Update node if existent
				// isOcc = tree.isNodeOccupied(node);
				tree.updateNodeLogOdds(node, logUpdateVal);
				// if(isOcc != tree.isNodeOccupied(node)){
					// counter = counter + 1;
				// }
			}
			

		}
	}
	// std::cout << "# of updated nodes: " << counter << std::endl;
}

//Function adapted from https://github.com/OctoMap/octomap_ros
void OctoClass::pointCloud2ToRayOctomap(const sensor_msgs::PointCloud2& cloud,
									   const tf::StampedTransform &tf_cam2world){

	sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
	sensor_msgs::PointCloud2ConstIterator<float> iter_y(cloud, "y");
	sensor_msgs::PointCloud2ConstIterator<float> iter_z(cloud, "z");

	//Get camera origin
	const tf::Vector3 v = tf_cam2world.getOrigin();
	static octomap::point3d camOrigin;
    camOrigin = octomap::point3d(v.getX(), v.getY(), v.getZ());
    const double minThresholdSquare = minRange*minRange;


	//Discretize point cloud
	octomap::Pointcloud octoCloud, inflatedOctocloud;
	octoCloud.reserve(cloud.height*cloud.width);
	inflatedOctocloud.reserve(cloud.height*cloud.width*(sphere.size()+1));
	octomap::KeySet endpoints, endpointsInflated;
	static octomap::point3d centralPoint, curPoint;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z){
      	//Points too close to origin of camera are not added
		if(this->VectorNormSquared(v.getX()-*iter_x, v.getY()-*iter_y,v.getZ()-*iter_z) < minThresholdSquare){
			continue;
		}

		// Check if the point is invalid
		if (!std::isnan (*iter_x) && !std::isnan (*iter_y) && !std::isnan (*iter_z)){
			
			//Create discretized octocloud 
			octomap::OcTreeKey k = tree.coordToKey(octomap::point3d(*iter_x, *iter_y, *iter_z));
			centralPoint = tree.keyToCoord(k);
			std::pair<octomap::KeySet::iterator,bool> ret = endpoints.insert(k);
			if (ret.second){ // insertion took place => k was not in set
				//Insert points in non-inflated octomap
				octoCloud.push_back(centralPoint);

				//Insert points in inflated octomap
				if(inflateMap){
					for(int j = 0; j < sphere.size(); j++){
						curPoint = centralPoint + octomap::point3d(sphere[j][0],sphere[j][1],sphere[j][2]);
						octomap::OcTreeKey key = treeInflated.coordToKey(curPoint);
						std::pair<octomap::KeySet::iterator,bool> ret = endpointsInflated.insert(key);
						if (ret.second){ // insertion took place => k was not in set
							inflatedOctocloud.push_back(curPoint);
						}
					}
				}

			}
		}
    }
 

    
    //Non-inflated octomap
    octomap::KeySet free_cells, occupied_cells;
    tree.computeUpdate(octoCloud, camOrigin, free_cells, occupied_cells, maxRange);
    for (octomap::KeySet::iterator it = free_cells.begin(); it != free_cells.end(); ++it) {
		tree.updateNode(*it, false);
    }
    for (octomap::KeySet::iterator it = occupied_cells.begin(); it != occupied_cells.end(); ++it) {
		tree.updateNode(*it, true);
    }

    //Inflated octomap
    octomap::KeySet free_cellsInflated, occupied_cellsInflated;
    tree.computeUpdate(inflatedOctocloud, camOrigin, free_cellsInflated, occupied_cellsInflated, maxRange);
    for (octomap::KeySet::iterator it = free_cellsInflated.begin(); it != free_cellsInflated.end(); ++it) {
		treeInflated.updateNode(*it, false);
    }
    for (octomap::KeySet::iterator it = occupied_cellsInflated.begin(); it != occupied_cellsInflated.end(); ++it) {
		treeInflated.updateNode(*it, true);
    }

   	
}



void OctoClass::fadeMemory(const double &rate){	//Rate at which this function is being called
	const double ClampLogMax = tree.getClampingThresMaxLog();
	const double ClampLogMin = tree.getClampingThresMinLog();
	const double occThres = tree.getOccupancyThres();
	// const double probLogHit = tree.getProbHitLog();
	const double probLogRangeObs = ClampLogMax - occThres;
	const double probLogRangeFree = ClampLogMin - occThres;
	const double FadingObsLogProbPerRun = -probLogRangeObs/(memoryTime*rate);
	const double FadingFreeLogProbPerRun = -probLogRangeFree/(memoryTime*rate);

	static bool isOcc;
	static octomap::OcTreeKey key;
	for(octomap::OcTree::leaf_iterator it = tree.begin_leafs(), 
		                               end= tree.end_leafs(); 
		                               it!= end; ++it){

		//Fade obstacles and free areas
		key = it.getKey();
		octomap::OcTreeNode* n = tree.search(key);
		isOcc = tree.isNodeOccupied(n);
		if(isOcc){
			tree.updateNodeLogOdds(n, FadingObsLogProbPerRun);
		}
		else{
			tree.updateNodeLogOdds(n, FadingFreeLogProbPerRun);
		}

		//Free nodes that are unknown
		if(isOcc != tree.isNodeOccupied(n)){ //If it was occupied then disoccupied, delete node
			tree.deleteNode(key,it.getDepth());
		}
	}
}

void OctoClass::inflateObstacles(const double &thickness){

	//Get all pixels in a sphere around the origin
	std::vector<Eigen::Vector3d> sphere;
	static Eigen::Vector3d XYZf;
	const int maxXYZ = (int)ceil(thickness/resolution);
	const float maxD = thickness*thickness;
	static float d_origin, D_plane;
	for(int x = -maxXYZ; x <= maxXYZ; x++){
		for(int y = -maxXYZ; y <= maxXYZ; y++){
			for(int z = -maxXYZ; z <= maxXYZ; z++){
				XYZf << x*resolution, y*resolution, z*resolution;

				d_origin = XYZf.dot(XYZf); //Distance from origin squared
				if(d_origin <= maxD){
					sphere.push_back(XYZf);					
				}

			}
		}
	}

	//Create inflated tree
	treeInflated.clear();
	const int nSphereNodes = sphere.size();
	static bool isCentralOcc, isOccInflated;
	static octomap::point3d centralPoint, curPoint;
	octomap::OcTreeNode *n, *nInflated;
	for(octomap::OcTree::leaf_iterator it = tree.begin_leafs(), 
		                               end= tree.end_leafs(); 
		                               it!= end; ++it){
		//Check occupancy of the current point
		centralPoint = it.getCoordinate();
		n = tree.search(centralPoint);
		isCentralOcc = tree.isNodeOccupied(n);

		//Populate the inflated map accordingly
		if(isCentralOcc){ //Populate the inflated with a sphere around this node
			for(int j = 0; j < nSphereNodes; j++){
				curPoint = centralPoint + octomap::point3d(sphere[j][0],sphere[j][1],sphere[j][2]);
				treeInflated.updateNode(curPoint,true);
			}
		}
		else{ //set as free if not uccupied already
			nInflated = treeInflated.search(centralPoint);
			if(nInflated == NULL){
				treeInflated.updateNode(centralPoint,false);
			}
			else if(!treeInflated.isNodeOccupied(nInflated)){
				treeInflated.updateNode(centralPoint,false);	
			}
		}
	}

}


//Adapted from https://github.com/OctoMap/octomap_mapping
void OctoClass::occupiedVisMarkers(visualization_msgs::MarkerArray* occupiedNodesVis){	//Publish occupied nodes
	// Markers: each marker array stores a set of nodes with similar size
	// visualization_msgs::MarkerArray occupiedNodesVis;
	occupiedNodesVis->markers.resize(treeDepth+1);
	const ros::Time rostime = ros::Time::now();

	//Get tree min and max
	double minX, minY, minZ, maxX, maxY, maxZ;
	tree.getMetricMin(minX, minY, minZ);
	tree.getMetricMax(maxX, maxY, maxZ);
	double colorFactor = 1.0;	//Define the gradient of colors

	//Publish all leafs from the tree
	for(octomap::OcTree::leaf_iterator it = tree.begin_leafs(), 
		                               end= tree.end_leafs(); 
		                               it!= end; ++it){
		if (tree.isNodeOccupied(*it)){
			//Get depth in the tree
			unsigned idx = it.getDepth();
			geometry_msgs::Point PointCenter;
			PointCenter.x = it.getX();
			PointCenter.y = it.getY();
			PointCenter.z = it.getZ();
			occupiedNodesVis->markers[idx].points.push_back(PointCenter);

			//Set color based on height
			double h = (1.0 - std::min(std::max((PointCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0))*colorFactor;
			occupiedNodesVis->markers[idx].colors.push_back(heightMapColor(h, 1.0));
		}
	}

	//Set marker properties
    for (unsigned i= 0; i < occupiedNodesVis->markers.size(); ++i){
		double size = tree.getNodeSize(i);

		occupiedNodesVis->markers[i].header.frame_id = "world";
		occupiedNodesVis->markers[i].header.stamp = rostime;
		occupiedNodesVis->markers[i].ns = "obstacleMap";
		occupiedNodesVis->markers[i].id = i;
		occupiedNodesVis->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
		occupiedNodesVis->markers[i].scale.x = size;
		occupiedNodesVis->markers[i].scale.y = size;
		occupiedNodesVis->markers[i].scale.z = size;

		if (occupiedNodesVis->markers[i].points.size() > 0)
			occupiedNodesVis->markers[i].action = visualization_msgs::Marker::ADD;
		else
			occupiedNodesVis->markers[i].action = visualization_msgs::Marker::DELETE;
    }


}

//Adapted from https://github.com/OctoMap/octomap_mapping
void OctoClass::freeVisMarkers(visualization_msgs::MarkerArray* freeNodesVis){	//Publish occupied nodes
	// Markers: each marker array stores a set of nodes with similar size
	// visualization_msgs::MarkerArray freeNodesVis;
	freeNodesVis->markers.resize(treeDepth+1);
	const ros::Time rostime = ros::Time::now();

	//Get tree min and max
	double minX, minY, minZ, maxX, maxY, maxZ;
	tree.getMetricMin(minX, minY, minZ);
	tree.getMetricMax(maxX, maxY, maxZ);
	double colorFactor = 1.0;	//Define the gradient of colors
	std_msgs::ColorRGBA color;
	// color = Color::Green();
	// color.a = 0.05;

	//Publish all leafs from the tree
	for(octomap::OcTree::leaf_iterator it = tree.begin_leafs(), 
		                               end= tree.end_leafs(); 
		                               it!= end; ++it){
		if (!tree.isNodeOccupied(*it)){
			//Get depth in the tree
			unsigned idx = it.getDepth();
			geometry_msgs::Point PointCenter;
			PointCenter.x = it.getX();
			PointCenter.y = it.getY();
			PointCenter.z = it.getZ();
			freeNodesVis->markers[idx].points.push_back(PointCenter);

			//Set color based on height
			double h = (1.0 - std::min(std::max((PointCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0))*colorFactor;
			freeNodesVis->markers[idx].colors.push_back(heightMapColor(h, 0.05));
			// freeNodesVis->markers[idx].colors.push_back(color);

		}
	}

	//Set marker properties
    for (unsigned i= 0; i < freeNodesVis->markers.size(); ++i){
		double size = tree.getNodeSize(i);

		freeNodesVis->markers[i].header.frame_id = "world";
		freeNodesVis->markers[i].header.stamp = rostime;
		freeNodesVis->markers[i].ns = "freeMap";
		freeNodesVis->markers[i].id = i;
		freeNodesVis->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
		freeNodesVis->markers[i].scale.x = size;
		freeNodesVis->markers[i].scale.y = size;
		freeNodesVis->markers[i].scale.z = size;

		if (freeNodesVis->markers[i].points.size() > 0)
			freeNodesVis->markers[i].action = visualization_msgs::Marker::ADD;
		else
			freeNodesVis->markers[i].action = visualization_msgs::Marker::DELETE;
    }


}

void OctoClass::inflatedOccVisMarkers(visualization_msgs::MarkerArray* occupiedNodesVis){	//Publish occupied nodes
	// Markers: each marker array stores a set of nodes with similar size
	// visualization_msgs::MarkerArray occupiedNodesVis;
	occupiedNodesVis->markers.resize(treeDepth+1);
	const ros::Time rostime = ros::Time::now();

	//Get tree min and max
	double minX, minY, minZ, maxX, maxY, maxZ;
	treeInflated.getMetricMin(minX, minY, minZ);
	treeInflated.getMetricMax(maxX, maxY, maxZ);
	double colorFactor = 1.0;	//Define the gradient of colors

	//Publish all leafs from the tree
	for(octomap::OcTree::leaf_iterator it = treeInflated.begin_leafs(), 
		                               end= treeInflated.end_leafs(); 
		                               it!= end; ++it){
		if (treeInflated.isNodeOccupied(*it)){
			//Get depth in the tree
			unsigned idx = it.getDepth();
			geometry_msgs::Point PointCenter;
			PointCenter.x = it.getX();
			PointCenter.y = it.getY();
			PointCenter.z = it.getZ();
			occupiedNodesVis->markers[idx].points.push_back(PointCenter);

			//Set color based on height
			double h = (1.0 - std::min(std::max((PointCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0))*colorFactor;
			occupiedNodesVis->markers[idx].colors.push_back(heightMapColor(h, 1.0));
		}
	}

	//Set marker properties
    for (unsigned i= 0; i < occupiedNodesVis->markers.size(); ++i){
		double size = treeInflated.getNodeSize(i);

		occupiedNodesVis->markers[i].header.frame_id = "world";
		occupiedNodesVis->markers[i].header.stamp = rostime;
		occupiedNodesVis->markers[i].ns = "inflatedObstacleMap";
		occupiedNodesVis->markers[i].id = i;
		occupiedNodesVis->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
		occupiedNodesVis->markers[i].scale.x = size;
		occupiedNodesVis->markers[i].scale.y = size;
		occupiedNodesVis->markers[i].scale.z = size;

		if (occupiedNodesVis->markers[i].points.size() > 0)
			occupiedNodesVis->markers[i].action = visualization_msgs::Marker::ADD;
		else
			occupiedNodesVis->markers[i].action = visualization_msgs::Marker::DELETE;
    }
}

void OctoClass::inflatedFreeVisMarkers(visualization_msgs::MarkerArray* freeNodesVis){	//Publish occupied nodes
	// Markers: each marker array stores a set of nodes with similar size
	// visualization_msgs::MarkerArray freeNodesVis;
	freeNodesVis->markers.resize(treeDepth+1);
	const ros::Time rostime = ros::Time::now();

	//Get tree min and max
	double minX, minY, minZ, maxX, maxY, maxZ;
	treeInflated.getMetricMin(minX, minY, minZ);
	treeInflated.getMetricMax(maxX, maxY, maxZ);
	double colorFactor = 1.0;	//Define the gradient of colors
	std_msgs::ColorRGBA color;
	// color = Color::Green();
	// color.a = 0.05;

	//Publish all leafs from the tree
	for(octomap::OcTree::leaf_iterator it = treeInflated.begin_leafs(), 
		                               end= treeInflated.end_leafs(); 
		                               it!= end; ++it){
		if (!treeInflated.isNodeOccupied(*it)){
			//Get depth in the tree
			unsigned idx = it.getDepth();
			geometry_msgs::Point PointCenter;
			PointCenter.x = it.getX();
			PointCenter.y = it.getY();
			PointCenter.z = it.getZ();
			freeNodesVis->markers[idx].points.push_back(PointCenter);

			//Set color based on height
			double h = (1.0 - std::min(std::max((PointCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0))*colorFactor;
			freeNodesVis->markers[idx].colors.push_back(heightMapColor(h, 0.05));
			// freeNodesVis->markers[idx].colors.push_back(color);

		}
	}

	//Set marker properties
    for (unsigned i= 0; i < freeNodesVis->markers.size(); ++i){
		double size = treeInflated.getNodeSize(i);

		freeNodesVis->markers[i].header.frame_id = "world";
		freeNodesVis->markers[i].header.stamp = rostime;
		freeNodesVis->markers[i].ns = "inflatedFreeMap";
		freeNodesVis->markers[i].id = i;
		freeNodesVis->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
		freeNodesVis->markers[i].scale.x = size;
		freeNodesVis->markers[i].scale.y = size;
		freeNodesVis->markers[i].scale.z = size;

		if (freeNodesVis->markers[i].points.size() > 0)
			freeNodesVis->markers[i].action = visualization_msgs::Marker::ADD;
		else
			freeNodesVis->markers[i].action = visualization_msgs::Marker::DELETE;
    }


}


double OctoClass::VectorNormSquared(const double &x, 
	                               const double &y, 
	                               const double &z){
	return x*x + y*y + z*z;
}

//Extracted from https://github.com/OctoMap/octomap_mapping
std_msgs::ColorRGBA OctoClass::heightMapColor(double h, double alpha) {

  std_msgs::ColorRGBA color;
  color.a = alpha;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
    case 6:
    case 0:
      color.r = v; color.g = n; color.b = m;
      break;
    case 1:
      color.r = n; color.g = v; color.b = m;
      break;
    case 2:
      color.r = m; color.g = v; color.b = n;
      break;
    case 3:
      color.r = m; color.g = n; color.b = v;
      break;
    case 4:
      color.r = n; color.g = m; color.b = v;
      break;
    case 5:
      color.r = v; color.g = m; color.b = n;
      break;
    default:
      color.r = 1; color.g = 0.5; color.b = 0.5;
      break;
  }

  return color;
}