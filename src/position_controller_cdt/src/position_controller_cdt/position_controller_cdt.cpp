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

  double error_path_angle = pathAngle - current_yaw;
  error_path_angle = constrainAngle(error_path_angle);

  // compute the P control output:
  double error_carr_yaw = goal_yaw - current_yaw;
  double error_carr_ang = constrainAngle(error_carr_yaw);

  // positional control
  // double x_error =  (cos(x_currentgoal) + sin(y_currentgoal)) - (cos(x_pos) + sin(y_pos));
  // double y_error =  (-sin(x_currentgoal) + cos(y_currentgoal)) - (-sin(x_pos) + cos(y_pos));

  double x_error_global = x_currentgoal - x_pos;
  double y_error_global = y_currentgoal - y_pos; 

  double x_error_rob = x_error_global * cos(current_yaw) + y_error_global * sin(current_yaw);
  double y_error_rob = y_error_global * -sin(current_yaw) + y_error_global * cos(current_yaw);

  double forward_gain_P = 1.1;
  double forward_gain_I = 0;
  //double max_speed = 0.4;
  //double high_speed = max_speed * (1- error_path_angle/(0.7*M_PI));

  // decrease gain when angular error is high
  double scal_forward_gain_P = forward_gain_P * (1- 0.9*abs(error_path_angle)/(0.5*M_PI));
  double forward_contr_speed = scal_forward_gain_P * x_error_rob - forward_gain_I * int_error_x;
  //double high_speed_switch = 0.1;
  //linear_forward_x = (x_error_rob < high_speed_switch) ? forward_contr_speed : high_speed;
  linear_forward_x = forward_contr_speed;

  double strafe_gain_P = 0.1;
  double strafe_gain_I = 0;
  linear_forward_y = strafe_gain_P * y_error_rob - strafe_gain_I * int_error_y;



  // angular trickery
  // path

  double path_ang_gain_P = 1.1; //0.95;
  // TODO maybe have minimum gain? Thresholding
  double scal_path_ang_gain = path_ang_gain_P ;//* 1.0 *abs(error_path_angle)/(0.6*M_PI);
  //double max_path_adjust_thresh = 0.3; //m
  double path_ang_gain_I = 0.1;
  //double ang_speed_path_contr = path_ang_gain_P * error_path_angle + path_ang_gain_I * int_error_path_ang;
  double ang_speed_path = path_ang_gain_P * error_path_angle + path_ang_gain_I * int_error_path_ang;

  //double ang_speed_path = (x_error_rob < max_path_adjust_thresh) ? 0 : ang_speed_path_contr;


  double carr_ang_gain_P = 0.6;
  double carr_ang_gain_I = 0.5;
  double ang_speed_head = carr_ang_gain_P * error_carr_ang + carr_ang_gain_I * int_error_carr_ang;

  double switch_thresh = 0.3;
  angular_velocity = (x_error_rob < switch_thresh) ? ang_speed_head : ang_speed_path;
  //angular_velocity = ang_speed_path;

  // x,y error in robot coordinates
  int_error_x += x_error_rob/(current_utime - past_utime);
  int_error_y += y_error_rob/(current_utime - past_utime);
  int_error_path_ang = error_path_angle/(current_utime - past_utime);
  int_error_carr_ang = error_carr_ang/(current_utime - past_utime);

  past_utime = current_utime;

  ///////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////

  std::cout << "current_yaw: " << current_yaw << ", raw error: " << error_carr_yaw
            << ", constrained error: " << error_carr_ang << ", des ang vel: " << angular_velocity 
            << ", error_path_angle" << error_path_angle << ", forward gain" << scal_forward_gain_P << std::endl;

  

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
