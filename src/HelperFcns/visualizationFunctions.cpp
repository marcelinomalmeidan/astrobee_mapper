

#include "visualizationFunctions.h"

// Overwrites the given properties of the marker array.
void setMarkerProperties(const std_msgs::Header& header, double life_time,
                         const visualization_msgs::Marker::_action_type& action,
                         visualization_msgs::MarkerArray* markers) {
  int count = 0;
  for (visualization_msgs::Marker& marker : markers->markers) {
    marker.header = header;
    marker.action = action;
    marker.id = count;
    marker.lifetime = ros::Duration(life_time);
    ++count;
  }
}


void deleteMarkersTemplate(
  const std::string& frame_id,
  visualization_msgs::MarkerArray* marker_array){
  
  visualization_msgs::Marker deleteMarker;
  deleteMarker.action = deleteMarker.DELETEALL;
  deleteMarker.scale.x = 0.1;
  deleteMarker.scale.y = 0.1;
  deleteMarker.scale.z = 0.1;
  deleteMarker.header.frame_id = frame_id;
  deleteMarker.ns = "";
  marker_array->markers.push_back(deleteMarker);
}

void setMarkersForDeletion(visualization_msgs::MarkerArray* marker_array){
  for(int i = 0; i < marker_array->markers.size(); i++){
    marker_array->markers[i].action = visualization_msgs::Marker::DELETE;
  }
}

void drawObstacleNodes(const std::vector<Eigen::Vector3d> Points, 
                       const std::string& frame_id,
                       const double resolution,
                       visualization_msgs::MarkerArray* marker_array) {
                   
  drawNodes(Points, frame_id, "obstacle", resolution, Color::Blue(), 0.1, marker_array);
}

void drawTreeNodes(const std::vector<Eigen::Vector3d> Points, 
                   const std::string& frame_id,
                   const double resolution,
                   visualization_msgs::MarkerArray* marker_array) {
                   
  drawNodes(Points, frame_id, "path", resolution, Color::Orange(), 0.1, marker_array);
}

void drawTreeNodes(
    const std::set<Eigen::Vector3d, vec3_compare> PointSet, 
    const std::string& frame_id,
    const double resolution,
    visualization_msgs::MarkerArray* marker_array){

  drawNodes(PointSet, frame_id, "path", resolution, Color::Orange(), 0.1, marker_array);
}

void drawCollidingNodes(const std::vector<Eigen::Vector3d> Points, 
                        const std::string& frame_id,
                        const double resolution,
                        visualization_msgs::MarkerArray* marker_array) {
                   
  drawNodes(Points, frame_id, "collision", resolution, Color::Red(), 0.9, marker_array);
}

void drawCollidingNodes(const std::set<Eigen::Vector3d, vec3_compare> PointSet, 
                        const std::string& frame_id,
                        const double resolution,
                        visualization_msgs::MarkerArray* marker_array){

  drawNodes(PointSet, frame_id, "collision", resolution, Color::Red(), 0.9, marker_array);
}

void drawNodes(const std::vector<Eigen::Vector3d> Points, 
               const std::string& frame_id,
               const std::string& ns, //namespace
               const double resolution,
               const std_msgs::ColorRGBA color,
               const double transparency, //0 -> transparent, 1 -> opaque
               visualization_msgs::MarkerArray* marker_array){
  marker_array->markers.clear();

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color = color;
  marker.color.a = transparency;
  marker.scale.x = resolution;
  marker.scale.y = resolution;
  marker.scale.z = resolution;
  marker.ns = ns;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.pose.orientation.w = 1.0;
  marker.header.seq = 0;
  marker.id = 0;

  //Get the number of requested waypoints
  int n_w = Points.size();

  for (size_t i = 0; i < n_w; ++i){
    geometry_msgs::Point NewPoint;
    NewPoint.x = Points[i](0);
    NewPoint.y = Points[i](1);
    NewPoint.z = Points[i](2);
    marker.points.push_back(NewPoint);
    // marker.pose.position = NewPoint;
    // marker_array->markers.push_back(marker);
    // i = i + 1;
  }
  marker_array->markers.push_back(marker);

  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  setMarkerProperties(header, 0.0, visualization_msgs::Marker::ADD,
                                marker_array);
}


void drawNodes(const std::set<Eigen::Vector3d, vec3_compare> PointSet, 
               const std::string& frame_id,
               const std::string& ns, //namespace
               const double resolution,
               const std_msgs::ColorRGBA color,
               const double transparency, //0 -> transparent, 1 -> opaque
               visualization_msgs::MarkerArray* marker_array){
  marker_array->markers.clear();

  visualization_msgs::Marker marker;
  marker.type = visualization_msgs::Marker::CUBE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color = color;
  marker.color.a = transparency;
  marker.scale.x = resolution;
  marker.scale.y = resolution;
  marker.scale.z = resolution;
  marker.ns = ns;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.pose.orientation.w = 1.0;
  marker.header.seq = 0;
  marker.id = 0;

  //Get the number of requested waypoints
  // int n_w = Points.size();

  Eigen::Vector3d CurPoint;
  // int i = 0;
  for (std::set<Eigen::Vector3d>::iterator it=PointSet.begin(); it!=PointSet.end(); ++it){
    CurPoint = *it;
    geometry_msgs::Point NewPoint;
    NewPoint.x = CurPoint(0);
    NewPoint.y = CurPoint(1);
    NewPoint.z = CurPoint(2);
    marker.points.push_back(NewPoint);
    // marker.pose.position = NewPoint;
    // marker_array->markers.push_back(marker);
    // i = i + 1;
  }
  marker_array->markers.push_back(marker);

  std_msgs::Header header;
  header.frame_id = frame_id;
  header.stamp = ros::Time::now();
  setMarkerProperties(header, 0.0, visualization_msgs::Marker::ADD,
                                marker_array);
}

void markerNode(const Eigen::Vector3d Point, 
                const std::string& frame_id,
                const std::string& ns, //namespace
                const double resolution,
                const std_msgs::ColorRGBA color,
                const double transparency, //0 -> transparent, 1 -> opaque
                const int seqNumber,
                visualization_msgs::Marker* marker){
  marker->type = visualization_msgs::Marker::CUBE;
  marker->action = visualization_msgs::Marker::ADD;
  marker->color = color;
  marker->color.a = transparency;
  marker->scale.x = resolution;
  marker->scale.y = resolution;
  marker->scale.z = resolution;
  marker->ns = ns;
  marker->header.frame_id = frame_id;
  marker->header.stamp = ros::Time::now();
  marker->pose.orientation.w = 1.0;

  geometry_msgs::Point position_msg;
  position_msg.x = Point(0);
  position_msg.y = Point(1);
  position_msg.z = Point(2);
  marker->pose.position = position_msg;
  marker->header.seq = seqNumber;
}