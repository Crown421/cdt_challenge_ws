#include "position_controller_cdt/position_controller_cdt.hpp"
#include "math.h"

PositionController::PositionController(){
  std::cout << "Finished setting up PositionController\n";
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
  



  // compute the P control output:
  double headingErrorRaw = current_yaw - goal_yaw;
  double headingError = constrainAngle(headingErrorRaw);

  //TODO needs tuning
  double angular_gain_p_ = 0.1;
  angular_velocity = -headingError * angular_gain_p_;

  // positional control
  double x_error =  (cos(x_currentgoal) + sin(y_currentgoal) - (cos(x_pos) + sin(y_pos));
  double y_error =  (-sin(x_currentgoal) + cos(y_currentgoal) - (-sin(x_pos) + cos(y_pos)) ;

  //double pos_error = sqrt( pow(x_error, 2.0) + pow(y_error, 2.0));

  double forward_gain = 1;
  double strafe_gain = 0.5; 
  //double rel_vel = forward_gain * pos_error;

  forward_speed = forward_gain * x_error;
  strafe_speed = strafe_gain * y_error;

  linear_forward_x = cos(current_yaw) * rel_vel;
  linear_forward_y = sin(current_yaw) * rel_vel;

  std::cout << "current_yaw: " << current_yaw << ", raw error: " << headingErrorRaw
            << ", constrained error: " << headingError << ", des ang vel: " << angular_velocity << std::endl;

  ///////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////

  // set outputs
  output_linear_velocity_ = Eigen::Vector3d(linear_forward_x, linear_forward_y, 0);
  output_angular_velocity_ = Eigen::Vector3d(0,0, angular_velocity) ;
  return SEND_COMMAND;
}
