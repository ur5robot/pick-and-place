
#include <ros/ros.h>

#include <string>
#include <vector>

#include "tabletop_object_detector/TabletopSegmentation.h"

/*! Simply pings the tabletop segmentation and recognition services and prints out the result.*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "ping_tabletop_segmentation");
  ros::NodeHandle nh;

  std::string service_name("/tabletop_segmentation");
  while ( !ros::service::waitForService(service_name, ros::Duration().fromSec(3.0)) && nh.ok() )
  {
    ROS_INFO("Waiting for service %s...", service_name.c_str());
  }
  if (!nh.ok()) exit(0);

  tabletop_object_detector::TabletopSegmentation segmentation_srv;
  if (!ros::service::call(service_name, segmentation_srv))
  {
    ROS_ERROR("Call to segmentation service failed");
    exit(0);
  }
  if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
  {
    ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);
    exit(0);
  }
  ROS_INFO("Segmentation service succeeded. Detected %d clusters", (int)segmentation_srv.response.clusters.size());
  if (segmentation_srv.response.clusters.empty()) exit(0);

  //also test segmenting with input table
  /*
  sleep(0.5);
  segmentation_srv.request.table = segmentation_srv.response.table;
  ROS_INFO("Re-segmenting with the same table shifted 10 cm closer to the robot");
  for(size_t i=0; i<segmentation_srv.request.table.convex_hull.vertices.size(); i++)
  {
    segmentation_srv.request.table.convex_hull.vertices[i].x -= .10;
  }
  if (!ros::service::call(service_name, segmentation_srv))
  {
    ROS_ERROR("Call to segmentation service failed");
    exit(0);
  }
  if (segmentation_srv.response.result != segmentation_srv.response.SUCCESS)
  {
    ROS_ERROR("Segmentation service returned error %d", segmentation_srv.response.result);
    exit(0);
  }
  ROS_INFO("Segmentation service with table input succeeded. Detected %d clusters", (int)segmentation_srv.response.clusters.size());
  */
  return true;
}
