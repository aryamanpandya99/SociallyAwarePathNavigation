#include <zone_marking/zone_marking.h>
#include <zone_marking/zoneMarking.h>
#include <zone_marking/mark_lines.h>
#include <pluginlib/class_list_macros.h>
#include<math.h>
#include <stdio.h>

PLUGINLIB_EXPORT_CLASS(marked_layer_namespace::markingLayer, costmap_2d::Layer)

    using costmap_2d::NO_INFORMATION;
    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::FREE_SPACE;

    namespace marked_layer_namespace
{
  markingLayer::markingLayer(){}
  void markingLayer::onInitialize()
  {
    ros::NodeHandle nh("~/"+name_);
    ros::NodeHandle pvt_nh("~");

    /*Advertise the two separate services , one to mark polygon zones
     and the other to mark zones represented by line segments*/
    mark_server_ = pvt_nh.advertiseService("zoneMarking", &markingLayer::zone_marking_call, this);
    line_server_ = pvt_nh.advertiseService("mark_lines", &markingLayer::mark_lines_cb, this);

    current_ = true;

    //set up dynamic reconfigure to enable/disable the costmap layer
    dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
    dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(&markingLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }

  //Callback to dynamic reconfigure
  void markingLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
  {
    enabled_ = config.enabled;
  }

  void markingLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y)
  {
    if (!enabled_)
      return;

    //this is still hard coded, change this
    mark_x_ = 100;
    mark_y_ = 100;

    *min_x = std::min(*min_x, 0.0);
    *min_y = std::min(*min_y, 0.0);
    *max_x = std::max(*max_x, 10.0);
    *max_y = std::max(*max_y, 10.0);
  }

  bool markingLayer::zone_marking_call(zone_marking::zoneMarking::Request &req,zone_marking::zoneMarking::Response &res )
  {

    geometry_msgs::PolygonStamped zoneData = req.shape;
    std_msgs::Bool answer;

    answer.data = true;
    res.outcome = answer;

    polygon_zones_map_.insert(std::pair<std::string, geometry_msgs::PolygonStamped>(req.name.data, req.shape));

    return true;
  }

  //Callback function for line marking service
  bool markingLayer::mark_lines_cb(zone_marking::mark_lines::Request &req, zone_marking::mark_lines::Response &res)
  {
    zone_marking::Lines new_lines_list = req.lines;
    int len = new_lines_list.segments.size();
    lines_list_.clear();
    for(int i = 0; i < len; i++)
    {
       zone_marking::segment newLine = new_lines_list.segments[i];
       std::string newName = new_lines_list.segments[i].name;
       lines_list_.insert(std::pair<std::string, zone_marking::segment> (newName, newLine));
    }

    return true;
  }

  //Given a PolygonStamped object, draws the polygon it represents within the costmap layer
  void markingLayer::mark_zone_from_shape(costmap_2d::Costmap2D& master_grid, geometry_msgs::PolygonStamped shape)
  {
    int len = shape.polygon.points.size();
    costmap_2d::Costmap2D* costmap = layered_costmap_->getCostmap();
    std::vector<geometry_msgs::Point32> map_polygon;

    /*The following loop extracts all the points representing the polygon
      and stores them in a point vector*/

    for (uint i = 0; i < len; ++i)
    {
      geometry_msgs::Point32 loc;
      int xVal = shape.polygon.points[i].x;
      int yVal = shape.polygon.points[i].y;
      loc.x = xVal;
      loc.y = yVal;
      map_polygon.push_back(loc);
    }

    //Rasterize polygon
    std::vector<geometry_msgs::Point32> polygon_cells;
    rasterizePolygon(map_polygon, polygon_cells);

    //Given the final vector, draw the shape within the costmap layer
    for (int i = 0; i < polygon_cells.size(); i++)
    {
      int k = polygon_cells[i].x;
      int mx = polygon_cells[i].x;
      int my = polygon_cells[i].y;
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
  }

  void markingLayer::updateCosts(costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i,
          int max_j)

   {
      if (!enabled_){
        return;
      }

      std::map<std::string, geometry_msgs::PolygonStamped>::iterator it;

      //Update costs based on zones hashmap
      for(it = polygon_zones_map_.begin(); it != polygon_zones_map_.end(); it++)
      {
	    mark_zone_from_shape(master_grid, it->second);
      }
      std::map<std::string, zone_marking::segment>::iterator it2;

      //Update costs based on lines hashmap
      for(it2 = lines_list_.begin(); it2 != lines_list_.end(); it2++)
      {
        mark_line(master_grid, it2->second);
      }
      return;
  }

  void markingLayer::mark_line(costmap_2d::Costmap2D& master_grid, zone_marking::segment & lineSeg)
  {
    std::vector<geometry_msgs::Point32> line_cells;
    int len = lineSeg.points.size();

    if(lineSeg.thickness < 1)
    {
      ROS_ERROR("Line thickness lower than threshold, aborting");
      return;
    }

    //If the line is 1 pixel thick, call the raytrace helper function
    else if(lineSeg.thickness == 1)
    {
      for(int i = 0; i < len-1; i++)
      {
        raytrace(lineSeg.points[i].x, lineSeg.points[i].y, lineSeg.points[i+1].x, lineSeg.points[i+1].y , line_cells);
      }
    }
    //If the line is thick, use the alternative helper function
    else
    {
      for(int i = 0; i < len-1; i++)
      {
        raytrace_thick(lineSeg.points[i].x, lineSeg.points[i].y, lineSeg.points[i+1].x, lineSeg.points[i+1].y,
        lineSeg.thickness, line_cells);
      }
    }

    len = line_cells.size();

    //Given a vector with all the line points, draw them within the costmap layer
    for(int k = 0; k < len; k++)
    {

      int j = line_cells[k].x;
      int mx = line_cells[k].x;
      int my = line_cells[k].y;
      master_grid.setCost(mx, my, LETHAL_OBSTACLE);
    }
  }

    /*
    The following function is adapted from https://github.com/rst-tu-dortmund/costmap_prohibition_layer
   */
  void markingLayer::rasterizePolygon(std::vector<geometry_msgs::Point32> & polygon, std::vector<geometry_msgs::Point32> & polygon_cells)
  {
    int len = polygon.size();
    if(len < 3)
    {
      ROS_ERROR("Less than 3 points provided, polygon construction not possible");
      return;
    }
    polygonOutlineCells(polygon, polygon_cells);

    int i = 0;
    geometry_msgs::Point32 swap;
    int count = 0;
    while(i < polygon_cells.size() - 1)
    {
      count++;
      if(polygon_cells[i].x > polygon_cells[i + 1].x)
      {
	swap = polygon_cells[i];
	polygon_cells[i] = polygon_cells[i + 1];
	polygon_cells[i + 1] = swap;
    if(i > 0)
    {
        --i;
    }
      }
      else
      {
        ++i;
      }
    }
    i = 0;

    geometry_msgs::Point32 min_pt;
    geometry_msgs::Point32 max_pt;
    double min_x = polygon_cells[0].x;
    double max_x = polygon_cells[(int)polygon_cells.size() -1].x;

        //walk through each column and mark cells inside the polygon
    for(int x = min_x; x <= max_x; ++x)
    {
      if(i >= (int)polygon_cells.size() - 1)
      {
        break;
      }

      if(polygon_cells[i].y < polygon_cells[i + 1].y)
      {
	min_pt = polygon_cells[i];
        max_pt = polygon_cells[i + 1];
      }
      else
      {
	min_pt = polygon_cells[i + 1];
        max_pt = polygon_cells[i];
      }

      i += 2;
      while(i < polygon_cells.size() && polygon_cells[i].x == x)
      {
	if(polygon_cells[i].y < min_pt.y)
        {
	  min_pt = polygon_cells[i];
	}
        else if(polygon_cells[i].y > max_pt.y)
	{
          max_pt = polygon_cells[i];
	}
        i++;
      }

      geometry_msgs::Point32 pt;
            //loop though cells in the column
      for(int y = min_pt.y; y < max_pt.y; ++y)
      {
	pt.x = x;
	pt.y = y;
	polygon_cells.push_back(pt);
      }
    }
  }

  void markingLayer::polygonOutlineCells(std::vector<geometry_msgs::Point32> & polygon, std::vector<geometry_msgs::Point32>& polygon_cells)
  {
    unsigned int len = polygon.size();
    for(unsigned int i = 0; i < len-1; i++ )
    {
      raytrace(polygon[i].x, polygon[i].y,polygon[i+1].x, polygon[i+1].y, polygon_cells);
    }
    if(len > 0)
    {
      unsigned int last_index = len - 1;
      raytrace(polygon[last_index].x, polygon[last_index].y,polygon[0].x, polygon[0].y, polygon_cells);
    }
  }

  /*
    The following function is adapted from https://github.com/rst-tu-dortmund/costmap_prohibition_layer
   */
  void markingLayer::raytrace(double x0, double y0, double x1, double y1, std::vector<geometry_msgs::Point32> &cells)
  {
    int dx = abs(x1-x0);
    int dy = abs(y1-y0);

    geometry_msgs::Point32 pt;

    pt.x = x0;
    pt.y = y0;
    int n = 1 + dx + dy;

    int x_inc = (x1 > x0) ? 1: -1;
    int y_inc = (y1 > y0) ? 1: -1;
    int error = dx - dy;
    dx*=2;
    dy*=2;

    for(; n > 0; n--)
    {
      cells.push_back(pt);

      if(error > 0)
      {
	pt.x += x_inc;
	error -= dy;
      }
      else
      {
	pt.y += y_inc;
	error += dx;
      }
    }
  }

  void markingLayer::raytrace_thick(double x0, double y0, double x1, double y1, int thickness, std::vector<geometry_msgs::Point32> &cells)
  {
    int dx = abs(x1-x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = abs(y1-y0);
    int sy = y0 < y1 ? 1 : -1;
    int err = dx-dy, e2, x2, y2;                          /* error value e_xy */
    float ed = dx+dy == 0 ? 1 : sqrt((float)dx*dx+(float)dy*dy);
    geometry_msgs::Point32 point_y, point_x, point_pixel;

    for (thickness = (thickness+1)/2; ; )
    {
      /* pixel loop */
      point_pixel.x = x0;
      point_pixel.y = y0;
      point_y.y = y0;
      point_x.x = x0;
      if(y0 < 0 || x0 < 0) break;
      cells.push_back(point_pixel);
      e2 = err; x2 = x0;
      if (2*e2 >= -dx)
      {                                           /* x step */
        for (e2 += dy, y2 = y0; e2 < ed*thickness && (y1 != y2 || dx > dy); e2 += dx)
	    {
	      y2 += sy;
	      point_x.y = y2;
	      point_x.x = x0;
	      if(y2 < 0 || x0 < 0) break;
	      cells.push_back(point_x);
	    }
        if (x0 == x1) break;
        e2 = err; err -= dy; x0 += sx;
      }
      if (2*e2 <= dy)
      {                                            /* y step */
        for (e2 = dx-e2; e2 < ed*thickness && (x1 != x2 || dx < dy); e2 += dy)
	    {
	      point_y.y = y0;
	      x2+=sx;
	      point_y.x = x2;
	      if(y0 < 0 || x2 < 0) break;
	      cells.push_back(point_y);
	    }
	    if (y0 == y1) break;
        err += dx; y0 += sy;
      }
    }
  }
}
