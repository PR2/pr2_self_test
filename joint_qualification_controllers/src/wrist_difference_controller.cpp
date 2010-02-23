/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "joint_qualification_controllers/wrist_difference_controller.h"

PLUGINLIB_REGISTER_CLASS(WristDifferenceController, 
                         joint_qualification_controllers::WristDifferenceController, 
                         pr2_controller_interface::Controller)

#define MAX_DATA_POINTS 120000

using namespace std;
using namespace joint_qualification_controllers;

WristDifferenceController::WristDifferenceController()
: flex_joint_(NULL),
  roll_joint_(NULL),
  robot_(NULL),
  data_sent_(false),
  wrist_data_pub_(NULL)
{
  wrist_test_data_.left_turn.time.resize(MAX_DATA_POINTS);
  wrist_test_data_.left_turn.flex_position.resize(MAX_DATA_POINTS);
  wrist_test_data_.left_turn.flex_effort.resize(MAX_DATA_POINTS);
  wrist_test_data_.left_turn.flex_cmd.resize(MAX_DATA_POINTS);
  wrist_test_data_.left_turn.roll_position.resize(MAX_DATA_POINTS);
  wrist_test_data_.left_turn.roll_effort.resize(MAX_DATA_POINTS);
  wrist_test_data_.left_turn.roll_cmd.resize(MAX_DATA_POINTS);
  wrist_test_data_.left_turn.roll_velocity.resize(MAX_DATA_POINTS);

  wrist_test_data_.right_turn.time.resize(MAX_DATA_POINTS);
  wrist_test_data_.right_turn.flex_position.resize(MAX_DATA_POINTS);
  wrist_test_data_.right_turn.flex_effort.resize(MAX_DATA_POINTS);
  wrist_test_data_.right_turn.flex_cmd.resize(MAX_DATA_POINTS);
  wrist_test_data_.right_turn.roll_position.resize(MAX_DATA_POINTS);
  wrist_test_data_.right_turn.roll_effort.resize(MAX_DATA_POINTS);
  wrist_test_data_.right_turn.roll_cmd.resize(MAX_DATA_POINTS);
  wrist_test_data_.right_turn.roll_velocity.resize(MAX_DATA_POINTS);

  wrist_test_data_.flex_pid.resize(4);
  wrist_test_data_.roll_pid.resize(4);

  wrist_test_data_.arg_name.resize(10);
  wrist_test_data_.arg_value.resize(10);
  wrist_test_data_.arg_name[0] = "Flex Position";
  wrist_test_data_.arg_name[1] = "Roll Velocity";
  wrist_test_data_.arg_name[2] = "Roll Tolerance (%)";
  wrist_test_data_.arg_name[3] = "Roll SD Max (%)";
  wrist_test_data_.arg_name[4] = "Timeout";
  wrist_test_data_.arg_name[5] = "Left Effort";
  wrist_test_data_.arg_name[6] = "Right Effort";
  wrist_test_data_.arg_name[7] = "Flex Tolerance";
  wrist_test_data_.arg_name[8] = "Flex Max Value";
  wrist_test_data_.arg_name[9] = "Flex SD";

  wrist_test_data_.timeout = false;
  
  state_         = STARTING;
  starting_count = 0;
  roll_velocity_ = 0;
  flex_position_ = 0;
  initial_time_  = ros::Time(0);
  left_count_    = 0;
  right_count_   = 0;
  start_count_   = 0;
  // Assume 1KHz update rate
  timeout_ = MAX_DATA_POINTS / 1000;
}

WristDifferenceController::~WristDifferenceController()
{
  delete flex_controller_;
  delete roll_controller_;
}

bool WristDifferenceController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  ROS_ASSERT(robot);
  robot_ = robot;

  std::string roll_name;
  if (!n.getParam("roll_velocity_controller/joint", roll_name)){
    ROS_ERROR("Hysteresis Controller: No joint name found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!(roll_joint_ = robot->getJointState(roll_name)))
  {
    ROS_ERROR("WristDifferenceController could not find joint named \"%s\"\n", roll_name.c_str());
    return false;
  }
  ROS_INFO("Roll joint: %s", roll_name.c_str());
  ///\todo Make sure roll joint is continuous, abort if not

  if (!n.getParam("roll_velocity", roll_velocity_)){
    ROS_ERROR("Hysteresis Controller: No velocity found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  roll_velocity_ = fabs(roll_velocity_);

  std::string flex_name;
  if (!n.getParam("flex_position_controller/joint", flex_name)){
    ROS_ERROR("Hysteresis Controller: No joint name found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!(flex_joint_ = robot->getJointState(flex_name)))
  {
    ROS_ERROR("WristDifferenceController could not find joint named \"%s\"\n", flex_name.c_str());
    return false;
  }

  if (!n.getParam("flex_position", flex_position_)) {
    ROS_ERROR("Hysteresis Controller: No velocity found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("timeout", timeout_)){
    ROS_ERROR("Hysteresis Controller: No timeout found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("tolerance", tolerance_)){
    ROS_WARN("Parameter 'tolerance' is not set on namespace: %s.", 
             n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("sd_max", sd_max_)) {
    ROS_WARN("Parameter 'sd_max' is not set on namespace: %s.", 
             n.getNamespace().c_str());
    return false;
  }

  double left_effort, right_effort, flex_tolerance, flex_max, flex_sd;
  if (!n.getParam("left_effort", left_effort)) {
    ROS_WARN("Parameter 'left_effort' is not set on namespace: %s.", 
             n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("right_effort", right_effort)) {
    ROS_WARN("Parameter 'right_effort' is not set on namespace: %s.", 
             n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("flex_tolerance", flex_tolerance)) {
    ROS_WARN("Parameter 'flex_tolerance' is not set on namespace: %s.", 
             n.getNamespace().c_str());
    return false;
  }


  if (!n.getParam("flex_max", flex_max)) {
    ROS_WARN("Parameter 'flex_max' is not set on namespace: %s.", 
             n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("flex_sd", flex_sd)) {
    ROS_WARN("Parameter 'flex_sd' is not set on namespace: %s.", 
             n.getNamespace().c_str());
    return false;
  }

  initial_time_ = robot_->getTime();

  // Set values in test data output
  wrist_test_data_.flex_joint = flex_name;
  wrist_test_data_.roll_joint = roll_name;
  wrist_test_data_.arg_value[0] = flex_position_;
  wrist_test_data_.arg_value[1] = roll_velocity_;
  wrist_test_data_.arg_value[2] = tolerance_;
  wrist_test_data_.arg_value[3] = sd_max_;
  wrist_test_data_.arg_value[4] = timeout_;
  wrist_test_data_.arg_value[5] = left_effort;
  wrist_test_data_.arg_value[6] = right_effort;
  wrist_test_data_.arg_value[7] = flex_tolerance;
  wrist_test_data_.arg_value[8] = flex_max;
  wrist_test_data_.arg_value[9] = flex_sd;

  flex_controller_ = new controller::JointPositionController();
  ros::NodeHandle n_flex(n, "flex_position_controller");
  if (!flex_controller_->init(robot, n_flex)) return false;

  roll_controller_ = new controller::JointVelocityController();
  ros::NodeHandle n_roll(n, "roll_velocity_controller");
  if (!roll_controller_->init(robot, n_roll)) return false;

  // Get the gains, add them to test data
  double p, i, d, iClamp, imin;
  roll_controller_->getGains(p, i, d, iClamp, imin);
  wrist_test_data_.roll_pid[0] = p;
  wrist_test_data_.roll_pid[1] = i;
  wrist_test_data_.roll_pid[2] = d;
  wrist_test_data_.roll_pid[3] = iClamp;

  flex_controller_->getGains(p, i, d, iClamp, imin);
  wrist_test_data_.flex_pid[0] = p;
  wrist_test_data_.flex_pid[1] = i;
  wrist_test_data_.flex_pid[2] = d;
  wrist_test_data_.flex_pid[3] = iClamp;

  wrist_data_pub_.reset(new realtime_tools::RealtimePublisher<joint_qualification_controllers::WristDiffData>(n, "/test_data", 1, true));

  return true;
}

void WristDifferenceController::starting()
{
  roll_controller_->starting();
  flex_controller_->starting();

  initial_time_ = robot_->getTime();
}

void WristDifferenceController::update()
{
  // wait until the all joints calibrated
  if(!flex_joint_->calibrated_ || !roll_joint_->calibrated_)
  {
    return;
  }

  ros::Time time = robot_->getTime();
  flex_controller_->update();
  roll_controller_->update();
  
  // Timeout check
  if ((time - initial_time_).toSec() > timeout_ && state_ != ANALYZING && state_ != DONE) 
  {
    state_ = ANALYZING;
    wrist_test_data_.timeout = true;
    roll_controller_->setCommand(0.0);
  }

  switch (state_)
  {
  case STARTING:
    roll_controller_->setCommand(roll_velocity_);
    flex_controller_->setCommand(flex_position_);
    start_count_++;
    // Let it settle for 3sec before recording data
    if (start_count_++ > 3000)
    {
      initial_position_ = roll_joint_->position_;
      state_ = LEFT;
    }
    break;
  case LEFT:
    if (left_count_ < MAX_DATA_POINTS)
    {
      wrist_test_data_.left_turn.time         [left_count_] = time.toSec();
      wrist_test_data_.left_turn.flex_position[left_count_] = flex_joint_->position_;
      wrist_test_data_.left_turn.flex_effort  [left_count_] = flex_joint_->measured_effort_;
      wrist_test_data_.left_turn.flex_cmd     [left_count_] = flex_joint_->commanded_effort_;

      wrist_test_data_.left_turn.roll_position[left_count_] = roll_joint_->position_;
      wrist_test_data_.left_turn.roll_effort  [left_count_] = roll_joint_->measured_effort_;
      wrist_test_data_.left_turn.roll_cmd     [left_count_] = roll_joint_->commanded_effort_;
      wrist_test_data_.left_turn.roll_velocity[left_count_] = roll_joint_->velocity_;
      left_count_++;
    }

    if(fabs(roll_joint_->position_ - initial_position_) > 6.28 || left_count_ >= MAX_DATA_POINTS)
    {
      double right_vel = -1 * roll_velocity_;
      roll_controller_->setCommand(right_vel);
      initial_position_ = roll_joint_->position_;
      state_ = RIGHT;
    }
    break;
  case RIGHT:
    if (right_count_ < MAX_DATA_POINTS)
    {
      wrist_test_data_.right_turn.time         [right_count_] = time.toSec();
      wrist_test_data_.right_turn.flex_position[right_count_] = flex_joint_->position_;
      wrist_test_data_.right_turn.flex_effort  [right_count_] = flex_joint_->measured_effort_;
      wrist_test_data_.right_turn.flex_cmd     [right_count_] = flex_joint_->commanded_effort_;

      wrist_test_data_.right_turn.roll_position[right_count_] = roll_joint_->position_;
      wrist_test_data_.right_turn.roll_effort  [right_count_] = roll_joint_->measured_effort_;
      wrist_test_data_.right_turn.roll_cmd     [right_count_] = roll_joint_->commanded_effort_;
      wrist_test_data_.right_turn.roll_velocity[right_count_] = roll_joint_->velocity_;
      right_count_++;
    }

    if(fabs(roll_joint_->position_ - initial_position_) > 6.28 || right_count_ >= MAX_DATA_POINTS)
    {
      roll_controller_->setCommand(0.0);
      state_ = ANALYZING;
    }
    break;
  case ANALYZING:
    roll_controller_->setCommand(0.0);
    analysis();
    state_ = DONE;
    break;
  case DONE:
    roll_controller_->setCommand(0.0);
    if (!data_sent_)
      data_sent_ = sendData();
    break;
  }
}

void WristDifferenceController::analysis()
{
  // Resize if no points
  if (left_count_ == 0)
    left_count_ = 1; 

  if (right_count_ == 0)
    right_count_ = 1; 

  wrist_test_data_.left_turn.time.resize(left_count_);
  wrist_test_data_.left_turn.flex_position.resize(left_count_);
  wrist_test_data_.left_turn.flex_effort.resize(left_count_);
  wrist_test_data_.left_turn.roll_cmd.resize(left_count_);
  wrist_test_data_.left_turn.roll_position.resize(left_count_);
  wrist_test_data_.left_turn.roll_effort.resize(left_count_);
  wrist_test_data_.left_turn.roll_cmd.resize(left_count_);
  wrist_test_data_.left_turn.roll_velocity.resize(left_count_);

  wrist_test_data_.right_turn.time.resize(right_count_);
  wrist_test_data_.right_turn.flex_position.resize(right_count_);
  wrist_test_data_.right_turn.flex_effort.resize(right_count_);
  wrist_test_data_.right_turn.roll_cmd.resize(right_count_);
  wrist_test_data_.right_turn.roll_position.resize(right_count_);
  wrist_test_data_.right_turn.roll_effort.resize(right_count_);
  wrist_test_data_.right_turn.roll_cmd.resize(right_count_);
  wrist_test_data_.right_turn.roll_velocity.resize(right_count_);

  return;
}

bool WristDifferenceController::sendData()
{
  if (wrist_data_pub_->trylock())
  {
    joint_qualification_controllers::WristDiffData *out = &wrist_data_pub_->msg_;
    out->flex_joint = wrist_test_data_.flex_joint;
    out->roll_joint = wrist_test_data_.roll_joint;
    out->flex_pid   = wrist_test_data_.flex_pid;
    out->roll_pid   = wrist_test_data_.roll_pid;
    out->arg_name   = wrist_test_data_.arg_name;
    out->arg_value  = wrist_test_data_.arg_value;
    out->left_turn  = wrist_test_data_.left_turn;
    out->right_turn = wrist_test_data_.right_turn;
    out->timeout    = wrist_test_data_.timeout;

    wrist_data_pub_->unlockAndPublish();
    ROS_INFO("Data sent");
    return true;
  }
  return false;
}



