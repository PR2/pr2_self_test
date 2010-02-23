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

#ifndef COUNTERBALANCE_TEST_CONTROLLER_H
#define COUNTERBALANCE_TEST_CONTROLLER_H

/***************************************************/
/*! \class controller::CounterbalanceTestController
    \brief Counterbalance Test Controller

    This holds a joint in a set of locations, while
    dithering the joint and recording position and cmd.

*/
/***************************************************/


#include <vector>
#include <ros/ros.h>
#include <math.h>
#include <joint_qualification_controllers/CounterbalanceTestData.h>
#include <realtime_tools/realtime_publisher.h>
#include <pr2_controller_interface/controller.h>
#include <robot_mechanism_controllers/joint_position_controller.h>
#include <control_toolbox/dither.h>
#include <iostream>
#include <string>

namespace joint_qualification_controllers
{

class CounterbalanceTestController : public pr2_controller_interface::Controller
{

public:
  enum { STARTING, SETTLING, DITHERING, NEXT, DONE };

  CounterbalanceTestController();
  ~CounterbalanceTestController();

  /*!
   * \brief Functional way to initialize.
   * \param *robot The robot that is being controlled.
   * \param &n Node handle for parameters and services
   */
  bool init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  /*!
   * \brief Issues commands to the joint. Should be called at regular intervals
   */
  void update();

  void starting();

  bool sendData();
    
  bool done() { return state_ == DONE; }
  
  joint_qualification_controllers::CounterbalanceTestData cb_test_data_;

private:
  control_toolbox::Dither* lift_dither_;
  control_toolbox::Dither* flex_dither_;

  controller::JointPositionController *lift_controller_;
  controller::JointPositionController *flex_controller_;

  pr2_mechanism_model::JointState *flex_state_;    
  pr2_mechanism_model::JointState *lift_state_;    

  pr2_mechanism_model::RobotState *robot_;

  int starting_count_;

  int state_;
  
  ros::Time initial_time_;

  double settle_time_;
  ros::Time start_time_;
  int dither_points_;
  double timeout_;

  int dither_count_;

  uint lift_index_;
  uint flex_index_;

  bool data_sent_;

  boost::scoped_ptr<realtime_tools::RealtimePublisher<joint_qualification_controllers::CounterbalanceTestData> > cb_data_pub_;


};

}



#endif //COUNTERBALANCE_TEST_CONTROLLER_H
