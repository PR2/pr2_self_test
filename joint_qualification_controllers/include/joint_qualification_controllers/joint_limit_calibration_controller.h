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

/*!
 * \author Kevin Watts
 *
 * This calibrates a joint by moving it to its positive or negative joint
 * limit, instead of using the calibration flag.
 */

#ifndef JOINT_LIMIT_CALIBRATION_CONTROLLER
#define JOINT_LIMIT_CALIBRATION_CONTROLLER

#include "ros/ros.h"
#include "pr2_mechanism_model/robot.h"
#include "robot_mechanism_controllers/joint_velocity_controller.h"
#include "realtime_tools/realtime_publisher.h"
#include "std_msgs/Empty.h"

namespace joint_qualification_controllers
{

class JointLimitCalibrationController : public pr2_controller_interface::Controller
{
public:
  JointLimitCalibrationController();
  virtual ~JointLimitCalibrationController();

  virtual bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  virtual void update();

  bool calibrated() { return state_ == CALIBRATED; }
  void beginCalibration() {
    if (state_ == INITIALIZED)
      state_ = BEGINNING;
  }

protected:
  pr2_mechanism_model::RobotState* robot_;
  ros::NodeHandle node_;
  boost::scoped_ptr<realtime_tools::RealtimePublisher<std_msgs::Empty> > pub_calibrated_;
  ros::Time last_publish_time_;

  enum { INITIALIZED, BEGINNING, STARTING, STOPPING, CALIBRATED };
  int state_;
  int count_;
  int stop_count_;

  double search_velocity_;

  pr2_hardware_interface::Actuator *actuator_;
  pr2_mechanism_model::JointState *joint_;
  pr2_mechanism_model::Transmission *transmission_;

  controller::JointVelocityController vc_; 
};


}


#endif //JOINT_LIMIT_CALIBRATION_CONTROLLER
