/**
 * @file   cameraToWorld.cpp
 * @Author Joseph Shepley jls2303@columbia.edu
 * @date   December, 2016
 * @brief  publishes transform from the base of the robot to the Kinect camera
 *
 * Detailed description of file.
 */
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <fstream>

using namespace std;


int main(int argc, char** argv){
  ros::init(argc, argv, "cameraToWorld");
  ros::NodeHandle node;

  tf::TransformBroadcaster br;
  tf::Transform camera_to_world_transform;
  vector<double> tfvector;

  //read from calibration file, store tf in vector
  std::ifstream infile("kinect_calibration.txt");  
  double num;
  int i=0;
  while (infile >> num)
  {
    tfvector.push_back (num);
    cout<<tfvector[i];
    i++;
  }


  //publish transform from /world to /camera_link
  ros::Rate rate(10.0);
  while (node.ok()){
    camera_to_world_transform.setOrigin( tf::Vector3(tfvector[0],tfvector[1],tfvector[2]) );
    tf::Quaternion q(tfvector[3],tfvector[4],tfvector[5],tfvector[6]);
    camera_to_world_transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(camera_to_world_transform, ros::Time::now(), "/world", "/camera_link"));
    rate.sleep();
  }
  return 0;
}