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

#include "joint_qualification_controllers/motor_joint_calibration_controller.h"
#include "ros/time.h"
#include "pluginlib/class_list_macros.h"


PLUGINLIB_DECLARE_CLASS(joint_qualification_controllers, MotorJointCalibrationController, 
                        joint_qualification_controllers::MotorJointCalibrationController, pr2_controller_interface::Controller)

using namespace std;
using namespace joint_qualification_controllers;


MotorJointCalibrationController::MotorJointCalibrationController()
: robot_(NULL), last_publish_time_(0), joint_(NULL)
{
}

MotorJointCalibrationController::~MotorJointCalibrationController()
{
}

bool MotorJointCalibrationController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  robot_ = robot;
  node_ = n;

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

  // "Calibrated" topic
  pub_calibrated_.reset(
    new realtime_tools::RealtimePublisher<std_msgs::Empty>(node_, "calibrated", 1));

  // Calibrate at zero, basically
  joint_->calibrated_ = true;

  return true;
}


void MotorJointCalibrationController::update()
{
  assert(joint_);

  if (pub_calibrated_)
  {
    if (last_publish_time_ + ros::Duration(0.5) < robot_->getTime())
    {
      assert(pub_calibrated_);
      if (pub_calibrated_->trylock())
      {
        last_publish_time_ = robot_->getTime();
        pub_calibrated_->unlockAndPublish();
      }
    }
  }

}

