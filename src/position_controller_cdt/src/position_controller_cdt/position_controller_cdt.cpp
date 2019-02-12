#include "position_controller_cdt/position_controller_cdt.hpp"
#include "math.h"
#include <geometry_msgs/WrenchStamped.h>

PositionController::PositionController(){
  std::cout << "Finished setting up PositionController\n";
  past_utime = 0;
}

// takes Quaterniod coordinates, and writes the passed-by-reference roll, pitch and yaw
void PositionController::quat_to_euler(Eigen::Quaterniond q, double& roll, double& pitch, double& yaw) {
  const double q0 = q.w();
  const double q1 = q.x();
  const double q2 = q.y();
  const double q3 = q.z();
  roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2));
  pitch = asin(2*(q0*q2-q3*q1));
  yaw = atan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3));
}


// constrain angle to be -180:180 in radians
double PositionController::constrainAngle(double x){
    x = fmod(x + M_PI,2.0*M_PI);
    if (x < 0)
        x += 2.0*M_PI;
    return x - M_PI;
}


FOLLOWER_OUTPUT PositionController::computeControlCommand(Eigen::Isometry3d current_pose, int64_t current_utime){
  double linear_forward_x = 0;
  double linear_forward_y = 0;
  double angular_velocity = 0;

  // static double integral = 0;
  // Develop your controller here within the calls

  // EXAMPLE HEADING CONTROLLER CODE - ADD YOUR OWN POSITION + HEADING CONTROLLER HERE
  ///////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////

  // convert own orientation to Euler coords
  Eigen::Quaterniond q(current_pose.rotation());
  double current_roll, current_pitch, current_yaw;
  quat_to_euler(q, current_roll, current_pitch, current_yaw);

  Eigen::Quaterniond q_goal(current_goal_.rotation());
  double goal_roll, goal_pitch, goal_yaw;
  quat_to_euler(q_goal, goal_roll, goal_pitch, goal_yaw);

  //Eigen::Translation x(current_pose.translation());
  //Eigen::Translation x_goal(current_goal_.translation());
  //x.x()

  double x_pos = current_pose.translation().x();
  double y_pos = current_pose.translation().y();

  double x_currentgoal = current_goal_.translation().x();
  double y_currentgoal = current_goal_.translation().y();
  

  // add heading yaw difference: 
  // Eigen::ArrayXXd ang(1, 2);
  // ang(0, 0) = (y_currentgoal - y_pos);
  // ang(0, 1) = (x_currentgoal - x_pos);
  // std::cout << ang;

  // Eigen::ArrayBase pathAngle(Eigen::arg(ang));
  //std::cout << Eigen:arg(ang) << std::endl;
  double pathAngle = atan2((y_currentgoal - y_pos), (x_currentgoal - x_pos));

  double error_path_angle = current_yaw - pathAngle;
  error_path_angle = constrainAngle(error_path_angle);

  // compute the P control output:
  double error_carrot_yaw = current_yaw - goal_yaw;
  double headingError = constrainAngle(error_carrot_yaw);

  // positional control
  // double x_error =  (cos(x_currentgoal) + sin(y_currentgoal)) - (cos(x_pos) + sin(y_pos));
  // double y_error =  (-sin(x_currentgoal) + cos(y_currentgoal)) - (-sin(x_pos) + cos(y_pos));

  double x_error_global = x_currentgoal - x_pos;
  double y_error_global = y_currentgoal - y_pos; 

  //TODO needs tuning
  double angular_gain_p_ = 0.01;
  double ang_gain_raw = 0.5 * (x_error_global*x_error_global);
  double max_ang_gain = 0.7;
  double path_ang_gain = (ang_gain_raw < max_ang_gain) ? ang_gain_raw : max_ang_gain;
  angular_velocity = -headingError * angular_gain_p_- path_ang_gain * error_path_angle;

  //double pos_error = sqrt( pow(x_error, 2.0) + pow(y_error, 2.0));

  double forward_gain = 0.5;
  double strafe_gain = 0.5; 
  //double rel_vel = forward_gain * pos_error;

  double forward_speed = forward_gain * x_error_global;
  double strafe_speed = strafe_gain * y_error_global;

  double for_vel_mag = sqrt(forward_speed*forward_speed + strafe_speed*strafe_speed);
  double for_vel_angle = atan2(forward_speed, strafe_speed);

  double diffAngle = current_yaw - for_vel_angle;

  linear_forward_x = for_vel_mag*cos(diffAngle);
  linear_forward_y = 0.1*for_vel_mag*sin(diffAngle);

  std::cout << "current_yaw: " << current_yaw << ", raw error: " << error_carrot_yaw
            << ", constrained error: " << headingError << ", des ang vel: " << angular_velocity << std::endl;

  ///////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////

  // set outputs
  output_linear_velocity_ = Eigen::Vector3d(linear_forward_x, linear_forward_y, 0);
  output_angular_velocity_ = Eigen::Vector3d(0,0, angular_velocity) ;

  // geometry_msgs::WrenchStamped ws;
  // //ws.header = msg.header;
  // ws.header.frame_id = "base";
  // ws.wrench.force.x = output_linear_velocity_[0]; /// insert your variable names
  // ws.wrench.force.y = output_linear_velocity_[1];
  // ws.wrench.force.z = 0;
  // ws.wrench.torque.x = 0;
  // ws.wrench.torque.y = 0;
  // // for some reason rviz wont visualise this if negative
  // ws.wrench.torque.z = fabs( output_angular_velocity_[2]);

  // visualizeVelocityCombinedPub_.publish(ws);

  return SEND_COMMAND;
}
