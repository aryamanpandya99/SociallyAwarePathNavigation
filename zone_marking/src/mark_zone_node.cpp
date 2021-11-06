#include "ros/ros.h"
#include "zone_marking/zoneMarking.h"

bool add(zone_marking::zoneMarking::Request  &req,
         zone_marking::zoneMarking::Response &res)
{
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zone_marking_server");
  ros::NodeHandle n;
  ros::ServiceServer service = n.advertiseService("zoneMarking", add);
  ros::service::exists("zone_marking/zoneMarking", true);
  //ros::spin();

  return 0;
}
