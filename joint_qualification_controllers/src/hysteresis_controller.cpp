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

#include "joint_qualification_controllers/hysteresis_controller.h"
#include "pluginlib/class_list_macros.h"
#include "urdf_model/joint.h"

PLUGINLIB_EXPORT_CLASS(joint_qualification_controllers::HysteresisController,
                       pr2_controller_interface::Controller)

#define MAX_DATA_POINTS 120000

using namespace std;
using namespace joint_qualification_controllers;

HysteresisController::HysteresisController() : 
  joint_(NULL),
  robot_(NULL),
  data_sent_(false),
  hyst_pub_(NULL)
{
  test_data_.joint_name = "default joint";
  test_data_.time_up.resize(MAX_DATA_POINTS);
  test_data_.effort_up.resize(MAX_DATA_POINTS);
  test_data_.position_up.resize(MAX_DATA_POINTS);
  test_data_.velocity_up.resize(MAX_DATA_POINTS);

  test_data_.time_down.resize(MAX_DATA_POINTS);
  test_data_.effort_down.resize(MAX_DATA_POINTS);
  test_data_.position_down.resize(MAX_DATA_POINTS);
  test_data_.velocity_down.resize(MAX_DATA_POINTS);

  test_data_.arg_name.resize(14);
  test_data_.arg_value.resize(14);
  test_data_.arg_name[0] = "Min. Expected Effort";
  test_data_.arg_name[1] = "Max. Expected Effort";
  test_data_.arg_name[2] = "Minimum Position";
  test_data_.arg_name[3] = "Maximum Position";
  test_data_.arg_name[4] = "Velocity";
  test_data_.arg_name[5] = "Timeout";
  test_data_.arg_name[6] = "Max. Allowed Effort";
  test_data_.arg_name[7] = "Tolerance";
  test_data_.arg_name[8] = "SD Max";
  test_data_.arg_name[9] = "Slope";
  test_data_.arg_name[10] = "P Gain";
  test_data_.arg_name[11] = "I Gain";
  test_data_.arg_name[12] = "D Gain";
  test_data_.arg_name[13] = "I-Clamp";

  state_         = STOPPED;
  starting_count_ = 0;
  velocity_      = 0;
  initial_time_  = ros::Time(0);
  max_effort_    = 0;
  complete       = false;
  up_count_      = 0;
  down_count_    = 0;
}

HysteresisController::~HysteresisController()
{
  if (velocity_controller_) delete velocity_controller_;
}

bool HysteresisController::init( pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  assert(robot);
  robot_ = robot;

  std::string name;
  if (!n.getParam("velocity_controller/joint", name)){
    ROS_ERROR("Hysteresis Controller: No joint name found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!(joint_ = robot->getJointState(name)))
  {
    ROS_ERROR("HysteresisController could not find joint named \"%s\"\n", name.c_str());
    return false;
  }

  if (!n.getParam("velocity", velocity_)){
    ROS_ERROR("Hysteresis Controller: No velocity found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  velocity_ = velocity_ > 0 ? velocity_ : -1.0 * velocity_;

  if (!n.getParam("max_effort", max_effort_)){
    ROS_ERROR("Hysteresis Controller: No max effort found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  double min_expected, max_expected, max_pos, min_pos;

    if (!n.getParam("min_expected", min_expected)){
    ROS_ERROR("Hysteresis Controller: No min expected effort found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("max_expected", max_expected)){
    ROS_ERROR("Hysteresis Controller: No max expected effort found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("max_position", max_pos)){
    ROS_ERROR("Hysteresis Controller: No max position found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("min_position", min_pos)){
    ROS_ERROR("Hysteresis Controller: No min position found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("timeout", timeout_)){
    ROS_ERROR("Hysteresis Controller: No timeout found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  double tolerance, sd_max;
  if (!n.getParam("tolerance", tolerance)){
    ROS_WARN("Parameter 'tolerance' is not set on namespace: %s. Default is 0.20.", 
             n.getNamespace().c_str());
    // This is deprecated, so eventually "return false" will be here.
    tolerance = 0.20;
  }

  if (!n.getParam("sd_max", sd_max)) {
    ROS_WARN("Parameter 'sd_max' is not set on namespace: %s. Default is 0.20.", 
             n.getNamespace().c_str());
    // This is deprecated, so eventually "return false" will be here.
    sd_max = 0.20;
  }

  double slope;
  if (!n.getParam("slope", slope))
    slope = 0;

  initial_time_ = robot_->getTime();
  initial_position_ = joint_->position_;

  // Set values in test data output
  test_data_.joint_name = name;
  test_data_.arg_value[0] = min_expected;
  test_data_.arg_value[1] = max_expected;
  test_data_.arg_value[2] = min_pos;
  test_data_.arg_value[3] = max_pos;
  test_data_.arg_value[4] = velocity_;
  test_data_.arg_value[5] = timeout_;
  test_data_.arg_value[6] = max_effort_;
  test_data_.arg_value[7] = tolerance;
  test_data_.arg_value[8] = sd_max;
  test_data_.arg_value[9] = slope;

  velocity_controller_ = new controller::JointVelocityController();
  ros::NodeHandle n_vel(n, "velocity_controller");
  if (!velocity_controller_->init(robot, n_vel)) return false;

  // Get the gains, add them to test data
  double p, i, d, iClamp, imin;
  velocity_controller_->getGains(p, i, d, iClamp, imin);

  test_data_.arg_value[10] = p;
  test_data_.arg_value[11] = i;
  test_data_.arg_value[12] = d;
  test_data_.arg_value[13] = iClamp;

  hyst_pub_.reset(new realtime_tools::RealtimePublisher<joint_qualification_controllers::HysteresisData>(n, "/test_data", 1, true));

  return true;
}

void HysteresisController::starting()
{
  velocity_controller_->starting();

  initial_time_ = robot_->getTime();
  initial_position_ = joint_->position_;
}

void HysteresisController::update()
{
  if (!joint_->calibrated_)
    return;

  ros::Time time = robot_->getTime();
  velocity_controller_->update();

  // Timeout check.
  if ((time - initial_time_).toSec() > timeout_ && state_ != ANALYZING && state_ != DONE) 
  {
    state_ = ANALYZING;
    test_data_.arg_value[5] = -1;
    velocity_controller_->setCommand(0.0);
  }

  switch (state_)
  {
  case STOPPED:
    velocity_controller_->setCommand(-1.0 * velocity_);
    starting_count_ = 0;
    state_ = MOVING_HOME;
    break;
  case MOVING_HOME:
    starting_count_++;
    if (turn() and starting_count_ > 100)
    {
      velocity_controller_->setCommand(velocity_);
      state_ = MOVING_UP;
      starting_count_ = 0;
    }
    break;
  case MOVING_UP:
    starting_count_++;
    if (up_count_ < MAX_DATA_POINTS)
    {
      test_data_.time_up[up_count_] = time.toSec();
      test_data_.effort_up[up_count_] = joint_->measured_effort_;
      test_data_.position_up[up_count_] = joint_->position_;
      test_data_.velocity_up[up_count_] = joint_->velocity_;
      up_count_++;
    }
    
    if ((turn() and starting_count_ > 100) or up_count_ >= MAX_DATA_POINTS)
    {
      velocity_controller_->setCommand(-1.0 * velocity_);
      state_ = MOVING_DOWN;
      starting_count_ = 0;
    }
    break;
  case MOVING_DOWN:
    starting_count_++;
    if (down_count_ < MAX_DATA_POINTS)
    {
      test_data_.time_down[down_count_] = time.toSec();
      test_data_.effort_down[down_count_] = joint_->measured_effort_;
      test_data_.position_down[down_count_] = joint_->position_;
      test_data_.velocity_down[down_count_] = joint_->velocity_;
      down_count_++;
    }
    if ((turn() and starting_count_ > 100) or down_count_ >= MAX_DATA_POINTS)
    {
      velocity_controller_->setCommand(0.0);
      state_ = ANALYZING;
      starting_count_ = 0;
    }
    break;
  case ANALYZING:
    velocity_controller_->setCommand(0.0);
    analysis();
    state_ = DONE;
    break;
  case DONE:
    velocity_controller_->setCommand(0.0);
    if (!data_sent_)
      data_sent_ = sendData();
    break;
  }

}

bool HysteresisController::turn()
{
  if (joint_->joint_->type!=urdf::Joint::CONTINUOUS)
  {
    return (fabs(joint_->velocity_) < 0.001 && fabs(joint_->commanded_effort_) > max_effort_);
  }
  else
  {
    if (fabs(joint_->position_-initial_position_) > 6.28)
    {
      initial_position_ = joint_->position_;
      return true;
    }
    return false;
  }
}

void HysteresisController::analysis()
{
  // Resize if no points
  if (up_count_ == 0)
    up_count_ = 1; 
  if (down_count_ == 0)
    down_count_ = 1;

  test_data_.time_up.resize(up_count_);
  test_data_.effort_up.resize(up_count_);
  test_data_.position_up.resize(up_count_);
  test_data_.velocity_up.resize(up_count_);

  test_data_.time_down.resize(down_count_);
  test_data_.effort_down.resize(down_count_);
  test_data_.position_down.resize(down_count_);
  test_data_.velocity_down.resize(down_count_);

  return;
}

bool HysteresisController::sendData()
{
  if (hyst_pub_->trylock())
  {
    joint_qualification_controllers::HysteresisData *out = &hyst_pub_->msg_;
    out->joint_name = test_data_.joint_name;

    out->time_up = test_data_.time_up;
    out->effort_up = test_data_.effort_up;
    out->position_up = test_data_.position_up;
    out->velocity_up = test_data_.velocity_up;

    out->time_down = test_data_.time_down;
    out->effort_down = test_data_.effort_down;
    out->position_down = test_data_.position_down;
    out->velocity_down = test_data_.velocity_down;


    out->arg_name = test_data_.arg_name;
    out->arg_value = test_data_.arg_value;

    hyst_pub_->unlockAndPublish();
    return true;
  }
  return false;
}



