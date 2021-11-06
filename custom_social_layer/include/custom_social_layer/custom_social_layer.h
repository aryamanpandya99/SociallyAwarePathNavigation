#ifndef CUSTOM_SOCIAL_LAYER_H_
#define CUSTOM_SOCIAL_LAYER_H_

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <custom_social_layer/People.h>
#include <custom_social_layer/Person.h>
#include <custom_social_layer/PeopleParametersConfig.h>

//Defining the social layer namespace

namespace social_layer_namespace
{
  class SocialLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D
  {
  public:
    SocialLayer();
    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);
    bool isDiscretized()
    {
      return true;
    }
    //Struct defined for shearing purposes
    struct Coordinates{
      int x;
      int y;
    };
    Coordinates shear(int x, int y, double angle);

    virtual void matchSize();
    virtual void midptellipse(costmap_2d::Costmap2D & master_grid, int rx, int ry, int xOrigin, int yOrigin, double orientation);
    virtual void peopleCallback(custom_social_layer::People people_list);
    virtual void reconfigure_people_params(custom_social_layer::PeopleParametersConfig &config, uint32_t level);

  private:
    void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
    dynamic_reconfigure::Server<custom_social_layer::PeopleParametersConfig> *dsrv_;
    dynamic_reconfigure::Server<custom_social_layer::PeopleParametersConfig> *people_dsrv_;

    double mark_x_, mark_y_;
    std::vector<custom_social_layer::People> people_vector_;
    ros::Subscriber people_sub_;
    ros::NodeHandle n;

    //dynamically reconfigurable parameters
    int ellipse_height_, ellipse_width_, stretch_factor_;
  };
}
#endif
