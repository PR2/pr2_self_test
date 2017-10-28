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


#include "joint_qualification_controllers/head_position_controller.h"
#include "pluginlib/class_list_macros.h"


PLUGINLIB_EXPORT_CLASS(joint_qualification_controllers::HeadPositionController,
                       pr2_controller_interface::Controller)

using namespace std;
using namespace joint_qualification_controllers;

HeadPositionController::HeadPositionController()
  : robot_state_(NULL)
{}

HeadPositionController::~HeadPositionController()
{
  sub_command_.shutdown();
}

bool HeadPositionController::init(pr2_mechanism_model::RobotState *robot_state, ros::NodeHandle &n)
{
  node_ = n;

// get name link names from the param server
  if (!node_.getParam("pan_link_name", pan_link_name_)){
    ROS_ERROR("HeadPositionController: No pan link name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }
  if (!node_.getParam("tilt_link_name", tilt_link_name_)){
    ROS_ERROR("HeadPositionController: No tilt link name found on parameter server (namespace: %s)",
              node_.getNamespace().c_str());
    return false;
  }

  // test if we got robot pointer
  assert(robot_state);
  robot_state_ = robot_state;

  //initialize the joint position controllers

  ros::NodeHandle nh_pan(node_, "pan_controller");
  ros::NodeHandle nh_tilt(node_, "tilt_controller");
  head_pan_controller_.init(robot_state, nh_pan);
  head_tilt_controller_.init(robot_state, nh_tilt);


  // subscribe to head commands
  sub_command_ = node_.subscribe<sensor_msgs::JointState>("command", 1, &HeadPositionController::command, this);

  return true;
}

void HeadPositionController::starting()
{
  pan_out_ = head_pan_controller_.joint_state_->position_;
  tilt_out_ = head_tilt_controller_.joint_state_->position_;
  head_pan_controller_.starting();
  head_tilt_controller_.starting();
}

void HeadPositionController::update()
{
  // set position controller commands
  head_pan_controller_.command_ = pan_out_;
  head_tilt_controller_.command_ = tilt_out_;
  head_pan_controller_.update();
  head_tilt_controller_.update();
}

void HeadPositionController::command(const sensor_msgs::JointStateConstPtr& command_msg)
{
  // do not use assert to check user input!

  if (command_msg->name.size() != 2 || command_msg->position.size() != 2){
    ROS_ERROR("Head servoing controller expected joint command of size 2");
    return;
  }
  if (command_msg->name[0] == head_pan_controller_.joint_state_->joint_->name &&
      command_msg->name[1] == head_tilt_controller_.joint_state_->joint_->name)
  {
    pan_out_  = command_msg->position[0];
    tilt_out_ = command_msg->position[1];
  }
  else if (command_msg->name[1] == head_pan_controller_.joint_state_->joint_->name &&
           command_msg->name[0] == head_tilt_controller_.joint_state_->joint_->name)
  {
    pan_out_ = command_msg->position[1];
    tilt_out_ = command_msg->position[0];
  }
  else
  {
    ROS_ERROR("Head servoing controller received invalid joint command");
  }
}



