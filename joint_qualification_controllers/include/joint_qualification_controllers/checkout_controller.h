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

// Author: Kevin Watts

#ifndef _JOINT_QUALIFICATION_CONTROLLERS_CHECKOUT_CONTROLLER_H_
#define _JOINT_QUALIFICATION_CONTROLLERS_CHECKOUT_CONTROLLER_H_

/***************************************************/
/*! \class controller::CheckoutController
    \brief Checkout Controller

    This tests that all joints of a robot are calibrated
    and all actuators are enabled. It outputs a RobotData srv request
    to the /robot_checkout topic with relevant joint and actuator information.

*/
/***************************************************/

#include "ros/ros.h"
#include <string>
#include <math.h>
#include <boost/scoped_ptr.hpp>
#include "joint_qualification_controllers/RobotData.h"
#include "realtime_tools/realtime_publisher.h"
#include "pr2_controller_interface/controller.h"


namespace joint_qualification_controllers
{


/***************************************************/
/*! \class joint_qualification_controllers::CheckoutController
    \brief Checkout Controller checks state of PR2 mechanism

    Verifies that all robot joints are calibrated. Publishes state of actuators and joints.

  */
/***************************************************/

class CheckoutController : public pr2_controller_interface::Controller
{

public:
  enum { STARTING, LISTENING, ANALYZING, DONE};

  CheckoutController();
  ~CheckoutController() { }

  /*!
   * \brief Functional way to initialize.
   * \param *robot The robot that is being controlled.
   * \param &n NodeHandle of mechanism control
   */
  bool init( pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

  /*!
   * \brief Called when controller is started or restarted
   */
  void starting();
  
  /*!
   * \brief Checks joint, actuator status for calibrated and enabled.
   */
  void update();

  /*! 
   * \brief Sends data, returns true if sent
   */
  bool sendData();

  /*!
   * \brief Perform the test analysis
   */
  void analysis(double time, bool timeout = false);


private:
  pr2_mechanism_model::RobotState *robot_;   
  ros::Time initial_time_;  /**< Start time of the test. */
 
  double timeout_;

  joint_qualification_controllers::RobotData robot_data_;
  
  int state_;

  int joint_count_;
  int actuator_count_;

  bool done() { return state_ == DONE; }

  bool data_sent_;

  double last_publish_time_;

  // RT service call
  boost::scoped_ptr<realtime_tools::RealtimePublisher<
                      joint_qualification_controllers::RobotData> > robot_data_pub_;
};



}


#endif //  _JOINT_QUALIFICATION_CONTROLLERS_CHECKOUT_CONTROLLER_H_
