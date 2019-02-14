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
  // Original Goal
  // Position pos_goal(8.5,4.0);
  // Harder Goal
  Position pos_goal(14.5, 4.0);

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
  auto start_total = std::chrono::high_resolution_clock::now();
  std::cout << "start - carrot planner\n";

  // Compute distance to the goal:
  Position pos_robot( pose_robot.translation().head(2) );
  double current_dist_to_goal = (pos_goal - pos_robot).norm();
  std::cout << "current distance to goal: " << current_dist_to_goal << std::endl;

  // If within 1.5m of goal - stop walking
  if (current_dist_to_goal < 1){
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
  GridMapRosConverter::fromMessage(message, inputMap);
  // Apply filter chain.
  grid_map::GridMap outputMap;
  auto start_filter_chain = std::chrono::high_resolution_clock::now();
  if (!filterChain_.update(inputMap, outputMap)) {
    ROS_ERROR("Could not update the grid map filter chain!");
    return false;
  }
  auto elapsed_filter_chain = std::chrono::high_resolution_clock::now() - start_filter_chain;
  long long milliseconds_filter_chain = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_filter_chain).count();
  std::cout << "Filter chain took " << milliseconds_filter_chain << "ms." << std::endl;

  ////// Our LIDAR Code! ////////////////////////////////////

   // convert own orientation to Euler coords
  Eigen::Quaterniond rob_q(pose_robot.rotation());
  double current_roll, current_pitch, current_yaw;

  const double q0 = rob_q.w();
  const double q1 = rob_q.x();
  const double q2 = rob_q.y();
  const double q3 = rob_q.z();
  double rob_yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));


  ////////////// Parameters ///////////////////////////
  double length_back = 0.06;
  double length_side = 0.08; 
  double traverse_thresh = 0.95;
  double lineSearch_thre = 0.24;
  double max_carrot_dist = 1.4;
  double nose_pos = 0.03;
  Position rob_centre_pos;
  rob_centre_pos(0) = pos_robot(0) + nose_pos * cos(rob_yaw);
  rob_centre_pos(1) = pos_robot(1) + nose_pos * sin(rob_yaw);
  
  Position rob_centre_left_pos;
  rob_centre_left_pos(0) = pos_robot(0) - ( length_back * cos(rob_yaw) - (-length_side) * sin(rob_yaw)); 
  rob_centre_left_pos(1) = pos_robot(1) - ( length_back * sin(rob_yaw) + (-length_side) * cos(rob_yaw));

  Position rob_centre_right_pos;
  rob_centre_right_pos(0) = pos_robot(0) - ( length_back * cos(rob_yaw) - length_side * sin(rob_yaw)); 
  rob_centre_right_pos(1) = pos_robot(1) - ( length_back * sin(rob_yaw) + length_side * cos(rob_yaw));

  // Apply our own filtering
  auto start_custom_filtering = std::chrono::high_resolution_clock::now();

  cv::Mat traversableImage, erodeImage, smoothedImage;
  cv::Mat elevation, elevationInpainted, elevationSmooth, normalX, normalY, normalZ, slope, roughness;
  int erosion_size = 45;  
  cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(erosion_size, erosion_size));
  GridMapCvConverter::toImage<unsigned short, 1>(outputMap, "traversability", CV_16UC1,
 traversableImage);
  GridMapCvConverter::toImage<unsigned short, 1>(outputMap, "elevation", CV_16UC1, elevation);
  

  cv::erode(traversableImage, erodeImage, element);
  cv::blur(erodeImage, smoothedImage, cv::Size_<int>(15, 15));

  GridMapCvConverter::addLayerFromImage<unsigned short, 1>(smoothedImage, "eroded_traversability", outputMap, 0.0, 1.0);  
  auto elapsed_custom_filtering = std::chrono::high_resolution_clock::now() - start_custom_filtering;
  long long milliseconds_custom_filtering = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_custom_filtering).count();
  std::cout << "Custom filtering took " << milliseconds_custom_filtering << "ms." << std::endl;

  // Search grid for best carrot
  auto start_carrot_search = std::chrono::high_resolution_clock::now();  
 
  Position best_pos;
  Position current_pos;
  double best_dist = 1000;
  double current_dist = 0;
  double dist_from_robot = 0;
 
  double curr_travers = 0;
  bool carrot_found = false;
  for (grid_map::GridMapIterator iterator(outputMap); !iterator.isPastEnd(); ++iterator) {
    curr_travers = outputMap.at("eroded_traversability", *iterator);
    if (curr_travers > traverse_thresh) {
      outputMap.getPosition(*iterator, current_pos);

      double dist_from_robot = (current_pos - pos_robot).norm();
      if (dist_from_robot > max_carrot_dist) {
        continue;
      }

      // test left side
      bool leftlineNotTravers = false;
      for (grid_map::LineIterator literator(outputMap, rob_centre_left_pos, current_pos); !literator.isPastEnd(); ++literator){
        if (outputMap.at("eroded_traversability", *literator) < lineSearch_thre){
          leftlineNotTravers = true;
          break;
        }
      }  
      if (leftlineNotTravers){
        continue;
      } 

      // test right side
      bool rightlineNotTravers = false;
      for (grid_map::LineIterator literator(outputMap, rob_centre_right_pos, current_pos); !literator.isPastEnd(); ++literator){
        if (outputMap.at("eroded_traversability", *literator) < lineSearch_thre){
          rightlineNotTravers = true;
          break;
        }
      }  
      if (rightlineNotTravers){
        continue;
      } 

      // test front of robot
      // bool robotlineTravers = false;
      // for (grid_map::LineIterator literator(outputMap, rob_centre_pos, current_pos); !literator.isPastEnd(); ++literator){
      //   if (outputMap.at("eroded_traversability", *literator) < lineSearch_thre){
      //     robotlineTravers = true;
      //     break;
      //   }
      // }  
      // if (robotlineTravers){
      //   continue;
      // } 

      current_dist = (pos_goal - current_pos).norm();
      if (current_dist < best_dist) {
        carrot_found = true;
        best_dist = current_dist;
        best_pos = current_pos;
      }
    }
  }

  // best_pos = rob_centre_right_pos;
  auto elapsed_carrot_search = std::chrono::high_resolution_clock::now() - start_carrot_search;
  long long milliseconds_carrot_search = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_carrot_search).count();
  std::cout << "Carrot search took " << milliseconds_carrot_search << "ms." << std::endl;

  // Compute correct angle from next carrot
  Eigen::Quaterniond q;
  q = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
    * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
    * Eigen::AngleAxisd(atan2(best_pos(1)-pos_robot(1), best_pos(0)-pos_robot(0)), Eigen::Vector3d::UnitZ());
	
  static Eigen::Isometry3d previous_carrot = pose_chosen_carrot; 

  if (carrot_found == true) {
    pose_chosen_carrot.translation() = Eigen::Vector3d( best_pos(0),best_pos(1),0);
    pose_chosen_carrot.linear() = q.matrix();
    previous_carrot = pose_chosen_carrot;
  }
  else {
    pose_chosen_carrot = previous_carrot;
  }
;

  ////// End of our LIDAR code! ////////////////////////////////////


  // Publish filtered output grid map.
  grid_map_msgs::GridMap outputMessage;
  GridMapRosConverter::toMessage(outputMap, outputMessage);
  outputGridmapPub_.publish(outputMessage);
  if (verbose_) std::cout << "finished processing\n";

  auto elapsed_total = std::chrono::high_resolution_clock::now() - start_total;
  long long milliseconds_total = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed_total).count();
  std::cout << milliseconds_total << "ms: lidar finished\n\n";

  return true;
}

} /* namespace */
