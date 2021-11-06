#ifndef ZONE_MARKING_H_
#define ZONE_MARKING_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <zone_marking/segment.h>
#include "zone_marking/zoneMarking.h"
#include "zone_marking/lineMarking.h"
#include <zone_marking/mark_lines.h>
#include <dynamic_reconfigure/server.h>

namespace marked_layer_namespace
{

class markingLayer : public costmap_2d::Layer
{
public:
  markingLayer();

  virtual void onInitialize();
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                             double* max_y);
  virtual void updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j);
  virtual bool zone_marking_call(zone_marking::zoneMarking::Request &req,zone_marking::zoneMarking::Response &res);
  virtual bool line_marking_call(zone_marking::lineMarking::Request &req,zone_marking::lineMarking::Response &res);
  virtual bool mark_lines_cb(zone_marking::mark_lines::Request &req, zone_marking::mark_lines::Response &res);
  bool isDiscretized()
  {
    return true;
  }
  virtual void mark_line(costmap_2d::Costmap2D& master_grid, zone_marking::segment & lineSeg);
  virtual void mark_zone_from_shape(costmap_2d::Costmap2D& master_grid, geometry_msgs::PolygonStamped shape);
  virtual void rasterizePolygon(std::vector<geometry_msgs::Point32> & polygon, std::vector<geometry_msgs::Point32> & polygon_cells);
  virtual void polygonOutlineCells(std::vector<geometry_msgs::Point32> & polygon, std::vector<geometry_msgs::Point32>& polygon_cells);
  virtual void raytrace(double x0, double y0, double x1, double y1, std::vector<geometry_msgs::Point32> &cells);
  virtual void raytrace_thick(double x0, double y0, double x1, double y1, int thickness, std::vector<geometry_msgs::Point32> &cells);

private:
  void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
  double mark_x_, mark_y_;
  ros::ServiceServer mark_server_;
  ros::ServiceServer line_server_;
  ros::NodeHandle n;
  geometry_msgs::PolygonStamped latest_msg;
  geometry_msgs::PolygonStamped prev_msg;
  dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
  std::vector<geometry_msgs::PolygonStamped> polygon_zones_;
  std::map<std::string, geometry_msgs::PolygonStamped> polygon_zones_map_;
  std::map<std::string, zone_marking::segment> lines_list_;
};
}
#endif
