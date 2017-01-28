/**
 * @file   calibrate.cpp
 * @Author Joseph Shepley jls2303@columbia.edu
 * @date   December, 2016
 * @brief  writes a kinect_calibration file to be used in camera_to_world_tf_broadcaster.cpp
 *         this program takes in an ar_marker_<NUMBER> as a parameter 
 * Detailed description of file.
 */
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <fstream>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "calibrate");
  ros::NodeHandle node;

  tf::TransformListener listener1;   //listen to tf between ar_marker and camera link
  tf::TransformBroadcaster br;       //broadcast transform between ar_marker and camera link
  tf::TransformBroadcaster brboard;  //broadcast transform between tool0 and calibration board
  tf::Transform tool0toboard;
  vector<double> tfvector;

  //read from transform file, store tf in vector
  std::ifstream infile("tool0toboard.txt");  
  double num;
  int i=0;
  while (infile >> num)
  {
    tfvector.push_back (num);
    cout<<tfvector[i];
    i++;
  }
  tool0toboard.setOrigin( tf::Vector3(tfvector[0],tfvector[1],tfvector[2]) );
  tool0toboard.setRotation( tf::createQuaternionFromRPY(0,0,0) );

  //argv[1] is the marker i.e. ar_marker_4
  //May replace "/ar_marker_4" with argv[1] assuming correct parameter input
  ros::Rate rate(10.0);
  while (node.ok()){
      tf::StampedTransform transformL;
      br.sendTransform(tf::StampedTransform(tool0toboard, ros::Time::now(), "/tool0", "/board"));
      try{

        listener1.lookupTransform("/ar_marker_4", "/camera_link",
                                 ros::Time(0), transformL);
        br.sendTransform(tf::StampedTransform(transformL, ros::Time::now(), "/board", "/kinect"));
        
          cout<<"success: Detected marker   ";
          tf::StampedTransform transformC;
          ros::Time now = ros::Time::now();
          listener1.waitForTransform("/world", "/kinect",
                              now, ros::Duration(1.0));
          listener1.lookupTransform("/world", "/kinect",
                                 ros::Time(0), transformC);


          //write transform to a file
          ofstream myfile;
          myfile.open ("kinect_calibration.txt");
          myfile << transformC.getOrigin().x()<<"\n";
          myfile << transformC.getOrigin().y()<<"\n";
          myfile << transformC.getOrigin().z()<<"\n";
          myfile << transformC.getRotation().getX()<<"\n";
          myfile << transformC.getRotation().getY()<<"\n";
          myfile << transformC.getRotation().getZ()<<"\n";
          myfile << transformC.getRotation().getW()<<"\n";
          cout<<"success!!!: Wrote kinect_calibration file"<<endl;
          myfile.close();
        

      }
      catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
         ros::Duration(1.0).sleep();
        continue;
      }
  rate.sleep();
  }

  //broadcast that transform to world
  



  while (node.ok()){
    //tf::Transform transformB;
    //transformB.setOrigin( tf::Vector3(0.054, 0.168, 0.677) );
    //transformB.setRotation( tf::Quaternion(-0.454, -0.462, 0.578, -0.497) );
    //br.sendTransform(tf::StampedTransform(transformB, ros::Time::now(), "/tool0", "/kinect"));
    
    //no there is a frame added to the ur5 tree.
    //rosrun tf tf_echo /world /kinect      should produce tf_c
    rate.sleep();
  }

  //get tf from world to kinect

  return 0;
};
