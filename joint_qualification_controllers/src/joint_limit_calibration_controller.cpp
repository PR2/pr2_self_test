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

#include "joint_qualification_controllers/joint_limit_calibration_controller.h"
#include "ros/time.h"
#include "pluginlib/class_list_macros.h"

using namespace std;
using namespace joint_qualification_controllers;

PLUGINLIB_EXPORT_CLASS(joint_qualification_controllers::JointLimitCalibrationController,
                       pr2_controller_interface::Controller)

JointLimitCalibrationController::JointLimitCalibrationController()
: robot_(NULL), last_publish_time_(0), state_(INITIALIZED), 
  count_(0), stop_count_(0), search_velocity_(0), 
  actuator_(NULL), joint_(NULL), transmission_(NULL)
{
}

JointLimitCalibrationController::~JointLimitCalibrationController()
{
}

bool JointLimitCalibrationController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  assert(robot);
  node_ = n;
  robot_ = robot;

  // Joint
  std::string joint_name;
  if (!node_.getParam("joint", joint_name))
  {
    ROS_ERROR("No joint given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(joint_ = robot->getJointState(joint_name)))
  {
    ROS_ERROR("Could not find joint %s (namespace: %s)",
              joint_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  // Actuator
  std::string actuator_name;
  if (!node_.getParam("actuator", actuator_name))
  {
    ROS_ERROR("No actuator given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(actuator_ = robot->model_->getActuator(actuator_name)))
  {
    ROS_ERROR("Could not find actuator %s (namespace: %s)",
              actuator_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  // Transmission
  std::string transmission_name;
  if (!node_.getParam("transmission", transmission_name))
  {
    ROS_ERROR("No transmission given (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }
  if (!(transmission_ = robot->model_->getTransmission(transmission_name).get()))
  {
    ROS_ERROR("Could not find transmission %s (namespace: %s)",
              transmission_name.c_str(), node_.getNamespace().c_str());
    return false;
  }

  if (!node_.getParam("velocity", search_velocity_))
  {
    ROS_ERROR("Velocity value was not specified (namespace: %s)", node_.getNamespace().c_str());
    return false;
  }

  // Contained velocity controller
  if (!vc_.init(robot, node_))
    return false;

  // "Calibrated" topic
  pub_calibrated_.reset(
    new realtime_tools::RealtimePublisher<std_msgs::Empty>(node_, "calibrated", 1));

  return true;
}

void JointLimitCalibrationController::update()
{
  assert(joint_);
  assert(actuator_);

  switch (state_)
  {
  case INITIALIZED:
    state_ = BEGINNING;
    return;
  case BEGINNING:
    count_ = 0;
    joint_->calibrated_ = false;
    actuator_->state_.zero_offset_ = 0.0;
    vc_.setCommand(search_velocity_);
    state_ = STARTING;
    break;
  case STARTING:
    // Makes sure we start moving for a bit before checking if we've stopped.
    if (++count_ > 500)
    {
      count_ = 0;
      state_ = STOPPING;
    }
    break;
  case STOPPING:
    if (fabs(joint_->velocity_) < 0.001)
      stop_count_++;
    else
      stop_count_ = 0;

    if (stop_count_ > 250)
    {
      // Need to set zero offset correctly
      pr2_hardware_interface::Actuator a;
      pr2_mechanism_model::JointState j;
      std::vector<pr2_hardware_interface::Actuator*> fake_a;
      std::vector<pr2_mechanism_model::JointState*> fake_j;
      fake_a.push_back(&a);
      fake_j.push_back(&j);

      fake_a[0]->state_.position_ = actuator_->state_.position_;

      transmission_->propagatePosition(fake_a, fake_j);

      // What is the actuator position at the joint's max or min?
      double ref_position = 0;
      if (search_velocity_ < 0)
        ref_position = joint_->joint_->limits->lower;
      else
        ref_position = joint_->joint_->limits->upper;


      fake_j[0]->position_ = fake_j[0]->position_ - ref_position;

      transmission_->propagatePositionBackwards(fake_j, fake_a);

      actuator_->state_.zero_offset_ = fake_a[0]->state_.position_;
      state_ = CALIBRATED;
      joint_->calibrated_ = true;
      vc_.setCommand(0);
    }
    break;
  case CALIBRATED:
    if (pub_calibrated_)
    {
      if (last_publish_time_ + ros::Duration(0.5) < robot_->getTime())
      {
        if (pub_calibrated_->trylock())
        {
          last_publish_time_ = robot_->getTime();
          pub_calibrated_->unlockAndPublish();
        }
      }
    }
    break;
  }

  if (state_ != CALIBRATED)
    vc_.update();
}

