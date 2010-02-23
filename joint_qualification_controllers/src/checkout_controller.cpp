/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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

#include "joint_qualification_controllers/checkout_controller.h"
#include "pluginlib/class_list_macros.h"

PLUGINLIB_REGISTER_CLASS(CheckoutController, joint_qualification_controllers::CheckoutController, pr2_controller_interface::Controller)

using namespace std;
using namespace joint_qualification_controllers;

CheckoutController::CheckoutController() :
  robot_(NULL),
  data_sent_(false),
  robot_data_pub_(NULL)
{
  robot_data_.test_time = -1;
  robot_data_.num_joints = 0;
  robot_data_.num_actuators = 0;

  state_          = STARTING;
  joint_count_    = 0;
  actuator_count_ = 0;

  timeout_        = 30.0;
}

bool CheckoutController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  assert(robot);
  robot_ = robot;

  joint_count_ = robot_->joint_states_.size();
  robot_data_.num_joints = joint_count_;
  robot_data_.timeout = false;

  robot_data_.joint_data.resize(joint_count_);

  for (int i = 0; i < joint_count_; i++)
  {
    pr2_mechanism_model::JointState *js = &(robot_->joint_states_[i]);

    robot_data_.joint_data[i].index = i;
    robot_data_.joint_data[i].name = js->joint_->name;
    robot_data_.joint_data[i].is_cal = 0;
    robot_data_.joint_data[i].has_safety = js->joint_->safety && js->joint_->limits;

    // Assign type strings based on type
    switch (js->joint_->type)
    {
    case (urdf::Joint::FIXED):
      robot_data_.joint_data[i].type = "Fixed";
      break;
    case (urdf::Joint::REVOLUTE):
      robot_data_.joint_data[i].type = "Rotary";
      break;
    case (urdf::Joint::CONTINUOUS):
      robot_data_.joint_data[i].type = "Continuous";
      break;
    case (urdf::Joint::PRISMATIC):
      robot_data_.joint_data[i].type = "Prismatic";
      break;
    case (urdf::Joint::PLANAR):
      robot_data_.joint_data[i].type = "Planar";
      break;
    default:
      robot_data_.joint_data[i].type = "No type given!";
      break;
    }
  }

  // Assign actuators
  actuator_count_ = robot_->model_->hw_->actuators_.size();
  robot_data_.num_actuators = actuator_count_;
  robot_data_.actuator_data.resize(actuator_count_);

  pr2_hardware_interface::ActuatorMap::const_iterator it;
  int i = 0;
  for (it = robot_->model_->hw_->actuators_.begin();
       it != robot_->model_->hw_->actuators_.end();
       ++it, ++i)
  {
    pr2_hardware_interface::Actuator *actuator = it->second;

    robot_data_.actuator_data[i].index = i;
    robot_data_.actuator_data[i].name = actuator->name_;
    robot_data_.actuator_data[i].id = actuator->state_.device_id_;
  }

  n.param("timeout", timeout_, 30.0);

  initial_time_ = robot_->getTime();

  robot_data_pub_.reset(new realtime_tools::RealtimePublisher<joint_qualification_controllers::RobotData>(n, "/robot_checkout", 1, true));

  return true;
}

void CheckoutController::starting()
{
  initial_time_ = robot_->getTime();

  data_sent_ = false;
}

void CheckoutController::update()
{
  ros::Time time = robot_->getTime();
  bool cal = false;
  //bool enabled = false;

  // Timeout check.
  if ((time - initial_time_).toSec() > timeout_ && state_ != ANALYZING && state_ != DONE) 
  {
    analysis((time - initial_time_).toSec(), true);
    state_ = DONE;
  }

  switch (state_)
  {
  case STARTING:
    initial_time_ = robot_->getTime();
    state_ = LISTENING;
    break;
  case LISTENING:
    {
    cal = true;
    for (int i = 0; i < joint_count_; ++i)
    {
      // Check for caster wheel joints and fingers
      // Wheel joints and fingers don't calibrate, so don't wait for them
      if (robot_->joint_states_[i].joint_->name.find("wheel_joint") != string::npos)
        continue;

      if (robot_->joint_states_[i].joint_->type == urdf::Joint::FIXED)
        continue;

      if (robot_->joint_states_[i].joint_->name.find("finger") != string::npos)
        continue;
      if (robot_->joint_states_[i].joint_->name.find("gripper_float_joint") != string::npos)
        continue;
      if (robot_->joint_states_[i].joint_->name.find("accelerometer_joint") != string::npos)
        continue;
      if (robot_->joint_states_[i].joint_->name.find("gripper_palm_joint") != string::npos)
        continue;
      if (robot_->joint_states_[i].joint_->name.find("gripper_tool_joint") != string::npos)
        continue;


      // Base joint is a dummy joint used to set up visualization
      if (robot_->joint_states_[i].joint_->name == "base_joint")
        continue;

      if (!robot_->joint_states_[i].calibrated_)
      {
        cal = false;
        break;
      }
    }

    //    enabled = true;

    /*
    pr2_hardware_interface::ActuatorMap::const_iterator it;
    for (it = robot_->model_->hw_->actuators_.begin();
         it != robot_->model_->hw_->actuators_.end();
         ++it)
    {
      if (it->second->name_ == "led_projector")
        continue;

      if (!it->second->state_.is_enabled_)
      {
        enabled = false;
        break;
      }
    }
    */
    
    if (cal)
      state_ = ANALYZING;

    break;
    }
  case ANALYZING:
    analysis((time - initial_time_).toSec());
    state_ = DONE;
    break;
  case DONE:
    if (!data_sent_)
      data_sent_ = sendData();


    break;
  }

}

bool CheckoutController::sendData()
{
  if (robot_data_pub_->trylock())
  {
    joint_qualification_controllers::RobotData *out = &robot_data_pub_->msg_;
    out->test_time     = robot_data_.test_time;
    out->num_joints    = robot_data_.num_joints;
    out->num_actuators = robot_data_.num_actuators;

    out->joint_data.resize(robot_data_.num_joints);
    out->actuator_data.resize(robot_data_.num_actuators);

    for (int i = 0; i < joint_count_; i++)
    {
      out->joint_data[i].index      = robot_data_.joint_data[i].index;
      out->joint_data[i].name       = robot_data_.joint_data[i].name;
      out->joint_data[i].is_cal     = robot_data_.joint_data[i].is_cal;
      out->joint_data[i].has_safety = robot_data_.joint_data[i].has_safety;
      out->joint_data[i].type       = robot_data_.joint_data[i].type;
    }

    for (int i = 0; i < actuator_count_; i++)
    {
      out->actuator_data[i].index   = robot_data_.actuator_data[i].index;
      out->actuator_data[i].name    = robot_data_.actuator_data[i].name;
      out->actuator_data[i].id      = robot_data_.actuator_data[i].id;
    }

    robot_data_pub_->unlockAndPublish();

    return true;
  }
  return false;
}

void CheckoutController::analysis(double time, bool timeout)
{
  robot_data_.test_time = time;
  robot_data_.timeout = timeout;

  for (int i = 0; i < joint_count_; i++)
  {
    robot_data_.joint_data[i].is_cal = robot_->joint_states_[i].calibrated_;

    if (robot_->joint_states_[i].joint_->type == urdf::Joint::FIXED)
      robot_data_.joint_data[i].is_cal = true;
  }

  /*
  pr2_hardware_interface::ActuatorMap::const_iterator it;
  int i = 0;
  for (it = robot_->model_->hw_->actuators_.begin();
       it != robot_->model_->hw_->actuators_.end();
       ++it, ++i)
  {
   
    if (it->second->name_ == "led_projector")
      robot_data_.actuator_data[i].enabled = true;
    else 
      robot_data_.actuator_data[i].enabled = it->second->state_.is_enabled_;

  }
  */
}
