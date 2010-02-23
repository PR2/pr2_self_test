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

#ifndef WRIST_DIFFERENCE_CONTROLLER_H
#define WRIST_DIFFERENCE_CONTROLLER_H

/***************************************************/
/*! \class controller::HysteresisController
    \brief Hystersis Controller

    This tests the hysteresis of a joint using a
    velocity controller.

*/
/***************************************************/

#include <ros/ros.h>
#include <string>
#include <math.h>
#include <joint_qualification_controllers/WristDiffData.h>
#include <realtime_tools/realtime_publisher.h>
#include <pr2_controller_interface/controller.h>
#include <robot_mechanism_controllers/joint_velocity_controller.h>
#include <robot_mechanism_controllers/joint_position_controller.h>
#include <boost/scoped_ptr.hpp>
#include "pluginlib/class_list_macros.h"

namespace joint_qualification_controllers
{

class WristDifferenceController : public pr2_controller_interface::Controller
{

public:
  enum { STARTING, LEFT, RIGHT, ANALYZING, DONE};

  WristDifferenceController();
  ~WristDifferenceController();

  /*!
   * \brief Functional way to initialize.
   * \param *robot The robot that is being controlled.
   * \param &n NodeHandle of mechanism control
   */
  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  /*!
   * \brief Called when controller is started
   */
  void starting();

  /*!
   * \brief Perform the test analysis
   */
  void analysis();

  /*!
   * \brief Issues commands to the joint to perform hysteresis test.
   */
  void update();

  /*! 
   * \brief Sends data, returns true if sent
   */
  bool sendData();
  
  bool done() { return state_ == DONE; }
  

private:
  joint_qualification_controllers::WristDiffData wrist_test_data_;

  pr2_mechanism_model::JointState *flex_joint_;     
  pr2_mechanism_model::JointState *roll_joint_;     
  pr2_mechanism_model::RobotState *robot_;    
  controller::JointPositionController *flex_controller_;   
  controller::JointVelocityController *roll_controller_; 

  double flex_position_;
  double roll_velocity_;            /**< Velocity during the test. */
  ros::Time initial_time_;             /**< Start time of the test. */
  double initial_position_;
  int left_count_, right_count_, start_count_;

  double timeout_;
  double tolerance_;
  double sd_max_;

  int state_;
  int starting_count;

  bool data_sent_;

  // RT service call
  boost::scoped_ptr<realtime_tools::RealtimePublisher<joint_qualification_controllers::WristDiffData> > wrist_data_pub_;

};
}


#endif
