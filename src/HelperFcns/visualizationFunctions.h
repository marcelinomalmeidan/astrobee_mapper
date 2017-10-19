#ifndef _H_VIS_FCNS_
#define _H_VIS_FCNS_


#include <visualization_msgs/MarkerArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include "compareVec3.h"


//Come colors for visualization markers
class Color : public std_msgs::ColorRGBA {
 public:
  Color() : std_msgs::ColorRGBA() {}
  Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
  Color(double red, double green, double blue, double alpha) : Color() {
    r = red;
    g = green;
    b = blue;
    a = alpha;
  }

  static const Color White() { return Color(1.0, 1.0, 1.0); }
  static const Color Black() { return Color(0.0, 0.0, 0.0); }
  static const Color Gray() { return Color(0.5, 0.5, 0.5); }
  static const Color Red() { return Color(1.0, 0.0, 0.0); }
  static const Color Green() { return Color(0.0, 1.0, 0.0); }
  static const Color Blue() { return Color(0.0, 0.0, 1.0); }
  static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
  static const Color Orange() { return Color(1.0, 0.5, 0.0); }
  static const Color Purple() { return Color(0.5, 0.0, 1.0); }
  static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
  static const Color Teal() { return Color(0.0, 1.0, 1.0); }
  static const Color Pink() { return Color(1.0, 0.0, 0.5); }
};

// Overwrites the given properties of the marker array.
void setMarkerProperties(const std_msgs::Header& header, double life_time,
                         const visualization_msgs::Marker::_action_type& action,
                         visualization_msgs::MarkerArray* markers);

//Delete markers from a given array
void setMarkersForDeletion(visualization_msgs::MarkerArray* marker_array);

void deleteMarkersTemplate( //Delete all markers
  const std::string& frame_id,
  visualization_msgs::MarkerArray* marker_array);

void drawObstacleNodes(const std::vector<Eigen::Vector3d> Points, 
                       const std::string& frame_id,
                       const double resolution,
                       visualization_msgs::MarkerArray* marker_array);

void drawTreeNodes(
    const std::vector<Eigen::Vector3d> Points, 
    const std::string& frame_id,
    const double resolution,
    visualization_msgs::MarkerArray* marker_array);

void drawTreeNodes(
    const std::set<Eigen::Vector3d, vec3_compare> PointSet, 
    const std::string& frame_id,
    const double resolution,
    visualization_msgs::MarkerArray* marker_array);


void drawCollidingNodes(const std::vector<Eigen::Vector3d> Points, 
                        const std::string& frame_id,
                        const double resolution,
                        visualization_msgs::MarkerArray* marker_array);

void drawCollidingNodes(const std::set<Eigen::Vector3d, vec3_compare> PointSet, 
                        const std::string& frame_id,
                        const double resolution,
                        visualization_msgs::MarkerArray* marker_array);

void drawCollidingNodes(const std::vector<octomap::point3d> Points, 
                        const std::string& frame_id,
                        const double resolution,
                        visualization_msgs::MarkerArray* marker_array);

void drawNodes(const std::vector<Eigen::Vector3d> Points, 
               const std::string& frame_id,
               const std::string& ns, //namespace
               const double resolution,
               const std_msgs::ColorRGBA color,
               const double transparency, //0 -> transparent, 1 -> opaque
               visualization_msgs::MarkerArray* marker_array);

void drawNodes(const std::set<Eigen::Vector3d, vec3_compare> PointSet, 
               const std::string& frame_id,
               const std::string& ns, //namespace
               const double resolution,
               const std_msgs::ColorRGBA color,
               const double transparency, //0 -> transparent, 1 -> opaque
               visualization_msgs::MarkerArray* marker_array);

void drawNodes(const std::vector<octomap::point3d> Points, 
               const std::string& frame_id,
               const std::string& ns, //namespace
               const double resolution,
               const std_msgs::ColorRGBA color,
               const double transparency, //0 -> transparent, 1 -> opaque
               visualization_msgs::MarkerArray* marker_array);

void markerNode(const Eigen::Vector3d Point, 
                const std::string& frame_id,
                const std::string& ns, //namespace
                const double resolution,
                const std_msgs::ColorRGBA color,
                const double transparency, //0 -> transparent, 1 -> opaque
                const int seqNumber,
                visualization_msgs::Marker* marker);

#endif