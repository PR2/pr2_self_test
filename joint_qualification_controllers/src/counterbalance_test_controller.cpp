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

/* Author: Kevin Watts */


#include <joint_qualification_controllers/counterbalance_test_controller.h>
#include "pluginlib/class_list_macros.h"

PLUGINLIB_REGISTER_CLASS(CounterbalanceTestController, 
                         joint_qualification_controllers::CounterbalanceTestController, 
                         pr2_controller_interface::Controller)

using namespace std;
using namespace joint_qualification_controllers;

CounterbalanceTestController::CounterbalanceTestController() :
  lift_dither_(NULL),
  flex_dither_(NULL),
  lift_controller_(NULL),
  flex_controller_(NULL),
  robot_(NULL),
  initial_time_(0.0),
  start_time_(0.0),
  cb_data_pub_(NULL)
{
  timeout_ = 180;
  lift_index_ = 0;
  flex_index_ = 0;
  data_sent_ = false;

  cb_test_data_.arg_name.resize(23);
  cb_test_data_.arg_value.resize(23);
  cb_test_data_.arg_name[0] = "Settle Time";
  cb_test_data_.arg_name[1] = "Dither Points";
  cb_test_data_.arg_name[2] = "Timeout";
  cb_test_data_.arg_name[3] = "Lift Min";
  cb_test_data_.arg_name[4] = "Lift Max";
  cb_test_data_.arg_name[5] = "Lift Delta";
  cb_test_data_.arg_name[6] = "Flex Min";
  cb_test_data_.arg_name[7] = "Flex Max";
  cb_test_data_.arg_name[8] = "Flex Delta";

  // P/F params
  cb_test_data_.arg_name[9] = "Lift MSE";
  cb_test_data_.arg_name[10] = "Lift Avg Abs";
  cb_test_data_.arg_name[11] = "Lift Avg Effort";
  cb_test_data_.arg_name[12] = "Flex MSE";
  cb_test_data_.arg_name[13] = "Flex Avg Abs";
  cb_test_data_.arg_name[14] = "Flex Avg Effort";

  // PID's
  cb_test_data_.arg_name[15] = "Lift P";
  cb_test_data_.arg_name[16] = "Lift I";
  cb_test_data_.arg_name[17] = "Lift D";
  cb_test_data_.arg_name[18] = "Lift I Clamp";

  cb_test_data_.arg_name[19] = "Flex P";
  cb_test_data_.arg_name[20] = "Flex I";
  cb_test_data_.arg_name[21] = "Flex D";
  cb_test_data_.arg_name[22] = "Flex I Clamp";

  cb_test_data_.timeout_hit = false;
  cb_test_data_.lift_joint = "None";
  cb_test_data_.lift_joint = "";
  cb_test_data_.lift_amplitude = 0;
  cb_test_data_.flex_amplitude = 0;

  ///\todo Need PID's for lift, flex

  state_ = STARTING;

}

CounterbalanceTestController::~CounterbalanceTestController()
{
  if (lift_controller_) delete lift_controller_;
  if (flex_controller_) delete flex_controller_;
  if (flex_dither_) delete flex_dither_;
  if (lift_dither_) delete lift_dither_;
}

bool CounterbalanceTestController::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
  ROS_ASSERT(robot);
  robot_ = robot;

  // Lift joint
  string lift_joint;
  if (!n.getParam("lift/joint", lift_joint)){
    ROS_ERROR("CounterbalanceTestController: No lift joint name found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!(lift_state_ = robot->getJointState(lift_joint)))
  {
    ROS_ERROR("CounterbalanceTestController could not find lift joint named \"%s\"\n", lift_joint.c_str());
    return false;
  }
  cb_test_data_.lift_joint = lift_joint;

  lift_controller_ = new controller::JointPositionController();
  ros::NodeHandle n_lift(n, "lift");
  if (!lift_controller_->init(robot, n_lift)) return false;

  int num_flexs = 1;
  double flex_min = 0;
  double flex_max = 0;
  double flex_delta = 0;
  double flex_mse = 0;
  double flex_avg_abs = 0;
  double flex_avg_eff = 0;

  cb_test_data_.flex_test = false;

  // Flex joint
  string flex_ctrl_type;
  if (n.getParam("flex/type", flex_ctrl_type))
  {
    cb_test_data_.flex_test = true;

    std::string flex_joint;
    if (!n.getParam("flex/joint", flex_joint))
    {
      ROS_ERROR("CounterbalanceTestController is starting without a flex joint, but told to flex. Node namespace: %s",
               n.getNamespace().c_str());
      return false;
    }

    if (!(flex_state_ = robot->getJointState(flex_joint)))
    {
      ROS_ERROR("CounterbalanceTestController could not find flex joint named \"%s\"\n", flex_joint.c_str());
      return false;
    }
    cb_test_data_.flex_joint = flex_joint;

    // Flex range
    if (!n.getParam("flex/min", flex_min))
    {
      ROS_ERROR("CounterbalanceTestController: No min flex position found on parameter namespace: %s)",
                n.getNamespace().c_str());
      return false;
    }
    if (!n.getParam("flex/max", flex_max))
    {
      ROS_ERROR("CounterbalanceTestController: No max flex position found on parameter namespace: %s)",
                n.getNamespace().c_str());
      return false;
    }
    if (!n.getParam("flex/delta", flex_delta))
    {
    ROS_ERROR("CounterbalanceTestController: No flex delta found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
    }

    if (flex_max < flex_min)
    {
      ROS_ERROR("CounterbalanceTestController: Flex max position must be >= flex min position");
      return false;
    }
    if (flex_delta < 0)
    {
      ROS_ERROR("CounterbalanceTestController: \"flex_delta\" parameter must be >=0");
      return false;
    }

    num_flexs = (int)(((flex_max - flex_min) / flex_delta) + 1);

    // MSE, Avg. Abs. Effort
    if (!n.getParam("flex/mse", flex_mse))
    {
      ROS_ERROR("CounterbalanceTestController: No flex/mse (mean square effort) found on parameter namespace: %s)",
                n.getNamespace().c_str());
      return false;
    }
    
    if (!n.getParam("flex/avg_abs", flex_avg_abs))
    {
      ROS_ERROR("CounterbalanceTestController: No flex/avg_abs (average absolute effort) found on parameter namespace: %s)",
                n.getNamespace().c_str());
      return false;
    }

    if (!n.getParam("flex/avg_eff", flex_avg_eff))
    {
      ROS_ERROR("CounterbalanceTestController: No flex/avg_eff (average absolute effort) found on parameter namespace: %s)",
                n.getNamespace().c_str());
      return false;
    }

    double flex_amplitude;
    if (!n.getParam("flex/dither", flex_amplitude))
    {
      ROS_ERROR("CounterbalanceTestController: No flex amplitude found on parameter namespace: %s)",
                n.getNamespace().c_str());
      return false;
    }
    flex_dither_ = new control_toolbox::Dither; 
    if (!flex_dither_->init(flex_amplitude, 100)) return false;

    cb_test_data_.flex_amplitude = flex_amplitude;

    flex_controller_ = new controller::JointPositionController();
    ros::NodeHandle n_flex(n, "flex");
    if (!flex_controller_->init(robot, n_flex)) return false;

    ROS_INFO("Initializing CounterbalanceTestController as a lift and flex test. Namespace: %s", 
             n.getNamespace().c_str());
  }
  else
  {
    ROS_INFO("Initializing CounterbalanceTestController as a shoulder lift test only. Namespace: %s",
             n.getNamespace().c_str());
  }

  double lift_amplitude;
  if (!n.getParam("lift/dither", lift_amplitude))
  {
    ROS_ERROR("CounterbalanceTestController: No lift amplitude found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  lift_dither_ = new control_toolbox::Dither; 
  if (!lift_dither_->init(lift_amplitude, 100)) return false;

  cb_test_data_.lift_amplitude = lift_amplitude;

  // Lift range
  double lift_min, lift_max, lift_delta;
  if (!n.getParam("lift/min", lift_min))
  {
    ROS_ERROR("CounterbalanceTestController: No min lift position found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("lift/max", lift_max))
  {
    ROS_ERROR("CounterbalanceTestController: No max lift position found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (lift_max < lift_min)
  {
    ROS_ERROR("CounterbalanceTestController was given a shoulder lift maximum with values less than its minimum. Max: %f, Min %f", lift_max, lift_min);
    return false;
  }

  if (!n.getParam("lift/delta", lift_delta))
  {
    ROS_ERROR("CounterbalanceTestController: No lift delta found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (lift_delta < 0)
  {
    ROS_ERROR("CounterbalanceTestController was given a shoulder lift position delta <0. Delta given: %f", lift_delta);
    return false;
  }

  // Setting controller defaults
  if (!n.getParam("settle_time", settle_time_))
  {
    ROS_ERROR("CounterbalanceTestController: No settle time found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("dither_points", dither_points_))
  {
    ROS_ERROR("CounterbalanceTestController: No dither points param found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("timeout", timeout_))
  {
    ROS_ERROR("CounterbalanceTestController: No timeout found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  double lift_mse, lift_avg_abs, lift_avg_eff;
  if (!n.getParam("lift/mse", lift_mse))
  {
    ROS_ERROR("CounterbalanceTestController: No lift/mse (mean square effort) found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("lift/avg_abs", lift_avg_abs))
  {
    ROS_ERROR("CounterbalanceTestController: No lift/avg_abs (average absolute effort) found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  if (!n.getParam("lift/avg_eff", lift_avg_eff))
  {
    ROS_ERROR("CounterbalanceTestController: No lift/avg_eff (average absolute effort) found on parameter namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  // Set up array of lift, flex commands
  int num_lifts = 1;
  if (lift_delta > 0)
    num_lifts = (int)((lift_max - lift_min)/lift_delta + 1);
  ROS_ASSERT(num_lifts > 0);

  cb_test_data_.lift_data.resize(num_lifts);
  for (int i = 0; i < num_lifts; ++i)
  {
    cb_test_data_.lift_data[i].lift_position = (double)lift_min + lift_delta * i;
    cb_test_data_.lift_data[i].flex_data.resize(num_flexs);
    for (int j = 0; j < num_flexs; ++j)
    {
      cb_test_data_.lift_data[i].flex_data[j].flex_position = (double)flex_min + flex_delta * ((double)j);

      cb_test_data_.lift_data[i].flex_data[j].lift_hold.time.resize(dither_points_);
      cb_test_data_.lift_data[i].flex_data[j].lift_hold.position.resize(dither_points_);
      cb_test_data_.lift_data[i].flex_data[j].lift_hold.velocity.resize(dither_points_);
      cb_test_data_.lift_data[i].flex_data[j].lift_hold.effort.resize(dither_points_);

      cb_test_data_.lift_data[i].flex_data[j].flex_hold.time.resize(dither_points_);
      cb_test_data_.lift_data[i].flex_data[j].flex_hold.position.resize(dither_points_);
      cb_test_data_.lift_data[i].flex_data[j].flex_hold.velocity.resize(dither_points_);
      cb_test_data_.lift_data[i].flex_data[j].flex_hold.effort.resize(dither_points_);
    }
  }

  cb_test_data_.arg_value[0] = settle_time_;
  cb_test_data_.arg_value[1] = dither_points_;
  cb_test_data_.arg_value[2] = timeout_;
  cb_test_data_.arg_value[3] = lift_min;
  cb_test_data_.arg_value[4] = lift_max;
  cb_test_data_.arg_value[5] = lift_delta;
  cb_test_data_.arg_value[6] = flex_min;
  cb_test_data_.arg_value[7] = flex_max;
  cb_test_data_.arg_value[8] = flex_delta;

  // Mean square efforts, avg abs efforts
  cb_test_data_.arg_value[9] = lift_mse;
  cb_test_data_.arg_value[10] = lift_avg_abs;
  cb_test_data_.arg_value[11] = lift_avg_eff;

  if (cb_test_data_.flex_test)
  {
    cb_test_data_.arg_value[12] = flex_mse;
    cb_test_data_.arg_value[13] = flex_avg_abs;
    cb_test_data_.arg_value[14] = flex_avg_eff;
  }
  else
  {
    cb_test_data_.arg_value[12] = 0;
    cb_test_data_.arg_value[13] = 0;
    cb_test_data_.arg_value[14] = 0;
  }

  double p, i, d, i_clamp, dummy;
  lift_controller_->getGains(p, i, d, i_clamp, dummy);
  cb_test_data_.arg_value[15] = p;
  cb_test_data_.arg_value[16] = i;
  cb_test_data_.arg_value[17] = d;
  cb_test_data_.arg_value[18] = i_clamp;

  if (cb_test_data_.flex_test)
  {
    flex_controller_->getGains(p, i, d, i_clamp, dummy);
    cb_test_data_.arg_value[19] = p;
    cb_test_data_.arg_value[20] = i;
    cb_test_data_.arg_value[21] = d;
    cb_test_data_.arg_value[22] = i_clamp;
  }
  else
  {
    cb_test_data_.arg_value[19] = 0;
    cb_test_data_.arg_value[20] = 0;
    cb_test_data_.arg_value[21] = 0;
    cb_test_data_.arg_value[22] = 0;
  }

  cb_data_pub_.reset(new realtime_tools::RealtimePublisher<
                     joint_qualification_controllers::CounterbalanceTestData>(n, "/cb_test_data", 1, true));

  //ROS_INFO("Initialized CB controller successfully!");

  return true;
}

void CounterbalanceTestController::starting()
{
  initial_time_ = robot_->getTime();
}

void CounterbalanceTestController::update()
{
  // wait until the joints are calibrated to start
  if (!lift_state_->calibrated_)
    return;
  if (cb_test_data_.flex_test and !flex_state_->calibrated_)
    return;

  ros::Time time = robot_->getTime();

  if ((time - initial_time_).toSec() > timeout_ && state_ != DONE)
  {
    ROS_WARN("CounterbalanceTestController timed out during test. Timeout: %f.", timeout_);
    state_ = DONE;
    cb_test_data_.timeout_hit = true;
  }

  lift_controller_->update();
  if (cb_test_data_.flex_test) 
    flex_controller_->update();

  switch (state_)
  {
  case STARTING:
    {
      double lift_cmd = cb_test_data_.lift_data[lift_index_].lift_position;
      double flex_cmd = cb_test_data_.lift_data[lift_index_].flex_data[flex_index_].flex_position;

      // Set controllers
      lift_controller_->setCommand(lift_cmd);
      if (cb_test_data_.flex_test) flex_controller_->setCommand(flex_cmd);

      dither_count_ = 0;
      start_time_ = time;

      state_ = SETTLING;

      //ROS_INFO("Moving to position lift %f, flex %f", lift_cmd, flex_cmd);
      break;
    }
  case SETTLING:
    {
      if ((time - start_time_).toSec() > settle_time_)
      {
        state_ = DITHERING;
        start_time_ = time;
      }

      break;
    }
  case DITHERING:
    {
      // Add dither
      lift_state_->commanded_effort_ += lift_dither_->update();
      if (cb_test_data_.flex_test) flex_state_->commanded_effort_ += flex_dither_->update();

      // Lift
      cb_test_data_.lift_data[lift_index_].flex_data[flex_index_].lift_hold.time[dither_count_] = (time - start_time_).toSec();
      cb_test_data_.lift_data[lift_index_].flex_data[flex_index_].lift_hold.position[dither_count_] = lift_state_->position_;
      cb_test_data_.lift_data[lift_index_].flex_data[flex_index_].lift_hold.velocity[dither_count_] = lift_state_->velocity_;
      cb_test_data_.lift_data[lift_index_].flex_data[flex_index_].lift_hold.effort[dither_count_] = lift_state_->measured_effort_;
      
      // Flex
      cb_test_data_.lift_data[lift_index_].flex_data[flex_index_].flex_hold.time[dither_count_] = (time - start_time_).toSec();
      if (cb_test_data_.flex_test)
      {
        cb_test_data_.lift_data[lift_index_].flex_data[flex_index_].flex_hold.position[dither_count_] = flex_state_->position_;
        cb_test_data_.lift_data[lift_index_].flex_data[flex_index_].flex_hold.velocity[dither_count_] = flex_state_->velocity_;
        cb_test_data_.lift_data[lift_index_].flex_data[flex_index_].flex_hold.effort[dither_count_] = flex_state_->measured_effort_;
      }
      else
      {
        cb_test_data_.lift_data[lift_index_].flex_data[flex_index_].flex_hold.position[dither_count_] = 0;
        cb_test_data_.lift_data[lift_index_].flex_data[flex_index_].flex_hold.velocity[dither_count_] = 0;
        cb_test_data_.lift_data[lift_index_].flex_data[flex_index_].flex_hold.effort[dither_count_] = 0;
      }

      ++dither_count_;
      
      if (dither_count_ >= dither_points_)
      {
        state_ = NEXT;
      }
      break;
    }
  case NEXT:
    {
      // Increment flex, lift indices
      ++flex_index_;
      if (flex_index_ >= cb_test_data_.lift_data[0].flex_data.size())
      {
        flex_index_ = 0;
        lift_index_++;
      }
      if (lift_index_ >= cb_test_data_.lift_data.size())
      {
        state_ = DONE;
        break;
      }

      state_ = STARTING;
      break;
    }
  case DONE:
    {
      if (!data_sent_)
        data_sent_ = sendData();

      break;
    }
  }
}

bool CounterbalanceTestController::sendData()
{
  if (cb_data_pub_->trylock())
  {
    // Copy data and send
    joint_qualification_controllers::CounterbalanceTestData *out = &cb_data_pub_->msg_;

    out->lift_joint     = cb_test_data_.lift_joint;
    out->flex_joint     = cb_test_data_.flex_joint;
    out->lift_amplitude = cb_test_data_.lift_amplitude;
    out->flex_amplitude = cb_test_data_.flex_amplitude;
    out->timeout_hit    = cb_test_data_.timeout_hit;
    out->flex_test      = cb_test_data_.flex_test;
    out->arg_name       = cb_test_data_.arg_name;
    out->arg_value      = cb_test_data_.arg_value;

    out->lift_data      = cb_test_data_.lift_data;
    cb_data_pub_->unlockAndPublish();
    return true;
  }
  return false;
}

