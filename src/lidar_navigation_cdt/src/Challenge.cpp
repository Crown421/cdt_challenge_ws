/*
 * NavigationDemo.cpp
 *
 *  Created on: Aug 16, 2017
 *      Author: Peter Fankhauser
 *   Institute: ETH Zurich, Robotic Systems Lab
 *
 */

#include "grid_map_cdt/Challenge.hpp"
#include <tf_conversions/tf_eigen.h>
#include <grid_map_cv/grid_map_cv.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <cv.h>
#include <highgui.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <eigen_conversions/eigen_msg.h>

using namespace grid_map;
using namespace std::chrono;


namespace grid_map_demos {

NavigationDemo::NavigationDemo(ros::NodeHandle& nodeHandle, bool& success)
    : nodeHandle_(nodeHandle),
      filterChain_("grid_map::GridMap"),
      demoMode_(false)
{
  if (!readParameters()) {
    success = false;
    return;
  }

  subscriber_ = nodeHandle_.subscribe(inputTopic_, 1, &NavigationDemo::callback, this);
  listener_ = new tf::TransformListener();

  outputGridmapPub_ = nodeHandle_.advertise<grid_map_msgs::GridMap>("/filtered_map", 1, true);
  footstepPlanRequestPub_ = nodeHandle_.advertise<geometry_msgs::PoseStamped>("/footstep_plan_request", 10);
  actionPub_ = nodeHandle_.advertise<std_msgs::Int16>("/action_cmd", 10);

  // Setup filter chain.
  if (!filterChain_.configure(filterChainParametersName_, nodeHandle)) {
    ROS_ERROR("Could not configure the filter chain!");
    success = false;
    return;
  }
  
  success = true;

  verbose_ = false;
  verboseTimer_ = true;
  plannerEnabled_ = true; // start enabled
}


NavigationDemo::~NavigationDemo()
{
}


bool NavigationDemo::readParameters()
{
  if (!nodeHandle_.getParam("input_topic", inputTopic_)) {
    ROS_ERROR("Could not read parameter `input_topic`.");
    return false;
  }
  nodeHandle_.param("filter_chain_parameter_name", filterChainParametersName_, std::string("grid_map_filters"));
  
  nodeHandle_.param("demo_mode", demoMode_, true);
  if (demoMode_)
    ROS_INFO("In demo mode [%d]. will use a hard coded gridmap bag and robot pose", int(demoMode_) );
  else
    ROS_INFO("In live mode [%d]. will listen for poses continuously", int(demoMode_) );

  return true;
}


void NavigationDemo::tic(){
  lastTime_ = high_resolution_clock::now();
}


std::chrono::duration<double> NavigationDemo::toc(){
  auto nowTime = high_resolution_clock::now();
  duration<double> elapsedTime = duration_cast<milliseconds>(nowTime - lastTime_);
  lastTime_ = nowTime;
  // std::cout << elapsedTime.count() << "ms elapsed" << std::endl;    
  return elapsedTime;
}


void NavigationDemo::callback(const grid_map_msgs::GridMap& message)
{
  if (!plannerEnabled_){
    std::cout << "planner enabled. at the goal? grab a beer!\n";
    return;
  }

  // The all important position goal - get the robot there
  Position pos_goal(8.5,4.0);

  Eigen::Isometry3d pose_robot = Eigen::Isometry3d::Identity();
  if(demoMode_){ // demoMode

    Eigen::Vector3d robot_xyz = Eigen::Vector3d(0.0,0.0,0); //rpy
    Eigen::Vector3d robot_rpy = Eigen::Vector3d(0,0,0); //rpy

    pose_robot.setIdentity();
    pose_robot.translation() = robot_xyz;
    Eigen::Quaterniond motion_R = Eigen::AngleAxisd(robot_rpy(2), Eigen::Vector3d::UnitZ())
        * Eigen::AngleAxisd(robot_rpy(1), Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(robot_rpy(0), Eigen::Vector3d::UnitX()); // order is ypr

    pose_robot.rotate( motion_R );

  }else{ // online

    tf::StampedTransform transform;
    try {
        listener_->waitForTransform("/odom", "/base", ros::Time(0), ros::Duration(10.0) );
        listener_->lookupTransform("/odom", "/base", ros::Time(0), transform);
    } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
    }

    tf::transformTFToEigen (transform, pose_robot);
    if (verbose_) std::cout << pose_robot.translation().transpose() << " current pose_robot\n";
  }


  Eigen::Isometry3d pose_chosen_carrot = Eigen::Isometry3d::Identity();
  bool sendCommand = planCarrot(message, pose_robot, pos_goal, pose_chosen_carrot);

  if(sendCommand){
    // Send the carrot to the position controller
    geometry_msgs::PoseStamped m;
    m.header = message.info.header;
    tf::poseEigenToMsg (pose_chosen_carrot, m.pose);
    footstepPlanRequestPub_.publish(m);
  }

}

bool NavigationDemo::planCarrot(const grid_map_msgs::GridMap& message,
  Eigen::Isometry3d pose_robot, Position pos_goal,
  Eigen::Isometry3d& pose_chosen_carrot)
{
  std::cout << "start - carrot planner\n";
  tic();

  // Compute distance to the goal:
  Position pos_robot( pose_robot.translation().head(2) );
  double current_dist_to_goal = (pos_goal - pos_robot).norm();
  std::cout << "current distance to goal: " << current_dist_to_goal << std::endl;

  // If within 1.5m of goal - stop walking
  if (current_dist_to_goal < 1.5){
    // Determine a carrot pose: x and y from the above. z is the robot's height.
    // yaw in the direction of the carrot. roll,pitch zero
    Eigen::Vector4d carrot_relative_pose = pose_robot.matrix().inverse()*Eigen::Vector4d(pos_goal(0), pos_goal(1), 0, 1) ;
    double carrot_relative_theta = atan2(carrot_relative_pose(1),carrot_relative_pose(0));
    if (verbose_) std::cout << carrot_relative_pose.transpose() << " - relative carrot\n";
    if (verbose_) std::cout << carrot_relative_theta << " - relative carrot - theta\n";

    Eigen::Isometry3d pose_chosen_carrot_relative = Eigen::Isometry3d::Identity();
    pose_chosen_carrot_relative.translation() = Eigen::Vector3d( carrot_relative_pose(0),carrot_relative_pose(1),0);
    Eigen::Quaterniond motion_R = Eigen::AngleAxisd(carrot_relative_theta, Eigen::Vector3d::UnitZ()) // yaw
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) // pitch
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()); // roll

    pose_chosen_carrot_relative.rotate( motion_R );
    pose_chosen_carrot = pose_robot * pose_chosen_carrot_relative;
    std::cout << current_dist_to_goal << "m to goal. carrot is goal\n";
    // disable carrot planner
    plannerEnabled_ = false;

    // Send message to position_controller to start free gait action
    std_msgs::Int16 actionMsg;
    actionMsg.data = 1;
    ros::Duration(1.0).sleep();
    actionPub_.publish(actionMsg);

    return true;
  }


  // Convert message to map.
  GridMap inputMap;
  std::cout << "Converting message to input map" << std::endl;
  GridMapRosConverter::fromMessage(message, inputMap);
  // Apply filter chain.
  grid_map::GridMap outputMap;
  std::cout << "Applying filter chain" << std::endl;
  if (!filterChain_.update(inputMap, outputMap)) {
    ROS_ERROR("Could not update the grid map filter chain!");
    return false;
  }
  std::cout << "Finished applying filter chain" << std::endl;
  if (verboseTimer_) std::cout << toc().count() << "ms: filter chain\n";


  ////// Put your code here ////////////////////////////////////
  // traversability = 1 means traversable
  Position best_pos;
  Position current_pos;
  double best_dist = 1000;
  double current_dist = 0;
  double dist_from_robot = 0;

  //std::cout << "Downsizing output map" << std::endl;
  //grid_map::GridMapCvProcessing::changeResolution(outputMap, outputMap, outputMap.getResolution()*8);
  //std::cout << "Downsizing complete" << std::endl;

  cv::Mat originalImage, erodeImage;
  GridMapCvConverter::toImage<unsigned short, 1>(outputMap, "traversability", CV_16UC1, 0.0, 1.0, originalImage);
  cv::imwrite( "originalImage.png", originalImage );

  cv::blur(originalImage, erodeImage, cv::Size_<int>(100,100));
  
  GridMapCvConverter::addLayerFromImage<unsigned short, 1>(erodeImage, "traversability_mean", outputMap, 0.0, 1.0);

  int i = 0;
  std::cout << "Entering grid map iterator loop" << std::endl;
  for (grid_map::GridMapIterator iterator(outputMap); !iterator.isPastEnd(); ++iterator) {
    if (outputMap.at("traversability_mean", *iterator) > 0.9) {
      outputMap.getPosition(*iterator, current_pos);
      double dist_from_robot = (current_pos - pos_robot).norm();
      if (dist_from_robot < 2) {
        current_dist = (pos_goal - current_pos).norm();
        if (current_dist < best_dist) {
          best_dist = current_dist;
          best_pos = current_pos;
        }
      }
    }
    ++i;
  } 
  std::cout << "Final i: " << i << std::endl;
  std::cout << "next distance: " << best_dist << "\n";	
  
  //double q1, q2, q3, q4;
  //double euler_rot = Eigen::Euler(0,0,atan2(best_pos(0), best_pos(1)));
  //euler_to_quat(euler_rot, q1, q2 ,q3, q4)

  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(atan2(best_pos(1), best_pos(0)), Eigen::Vector3d::UnitZ());
	
  pose_chosen_carrot.translation() = Eigen::Vector3d( best_pos(0),best_pos(1),0);
  pose_chosen_carrot.linear() = q.matrix();

  ////// Put your code here ////////////////////////////////////


  // Publish filtered output grid map.
  grid_map_msgs::GridMap outputMessage;
  GridMapRosConverter::toMessage(outputMap, outputMessage);
  outputGridmapPub_.publish(outputMessage);
  if (verbose_) std::cout << "finished processing\n";
  if (verboseTimer_) std::cout << toc().count() << "ms: publish output\n";

  std::cout << "finish - carrot planner\n\n";
  

  // REMOVE THIS WHEN YOUR ARE DEVELOPING ----------------
  // create a fake carrot - replace with a good carrot
  //std::cout << "REPLACE FAKE CARROT!\n";
  //pose_chosen_carrot.translation() = Eigen::Vector3d(1.0,0,0);
  // REMOVE THIS -----------------------------------------

  return true;
}

} /* namespace */
