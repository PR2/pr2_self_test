/*
 * teleop_head
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the <ORGANIZATION> nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Tully Foote

#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include "ros/ros.h"
#include "joy/Joy.h"

#include "sensor_msgs/JointState.h"

class TeleopHead 
{
   public:
  double req_pan, req_tilt, max_pan, max_tilt, min_tilt;
  int axis_pan, axis_tilt;
  int deadman_button, passthrough_button;
  double pan_step, tilt_step;
  bool deadman_no_publish_;
  bool deadman_;

  ros::NodeHandle n_;
  ros::Publisher head_pub_;
  ros::Subscriber joy_sub_;

  TeleopHead(bool deadman_no_publish = false) : max_pan(0.6), max_tilt(1.4), min_tilt(0.4), pan_step(0.1), tilt_step(0.1), deadman_no_publish_(deadman_no_publish), deadman_(false) {}

  void init()
  {
    req_pan = req_tilt = 0;
    
    // Head pan/tilt parameters
    n_.param("max_pan", max_pan, max_pan);
    n_.param("max_tilt", max_tilt, max_tilt);
    n_.param("min_tilt", min_tilt, min_tilt);
    
    n_.param("tilt_step", tilt_step, tilt_step);
    n_.param("pan_step", pan_step, pan_step);
    
    n_.param("axis_pan", axis_pan, 0);
    n_.param("axis_tilt", axis_tilt, 2);

    n_.param("deadman_button", deadman_button, 0);
    
    ROS_DEBUG("tilt step: %.3f rad\n", tilt_step);
    ROS_DEBUG("pan step: %.3f rad\n", pan_step);
    
    ROS_DEBUG("axis_pan: %d\n", axis_pan);
    ROS_DEBUG("axis_tilt: %d\n", axis_tilt);
    
    ROS_DEBUG("deadman_button: %d\n", deadman_button);
    
    head_pub_ = n_.advertise<sensor_msgs::JointState>("head_controller/command", 1);

    joy_sub_ = n_.subscribe("joy", 10, &TeleopHead::joy_cb, this);
  }


  ~TeleopHead() {  }


  void joy_cb(const joy::Joy::ConstPtr& joy_msg)
  {
    deadman_ = (((unsigned int)deadman_button < joy_msg->get_buttons_size()) && joy_msg->buttons[deadman_button]);

    if (!deadman_)
      return;

    if((axis_pan >= 0) && (((unsigned int)axis_pan) < joy_msg->get_axes_size()))
    {
      req_pan += joy_msg->axes[axis_pan] * pan_step;
      req_pan = std::max(std::min(req_pan, max_pan), -max_pan);
    }
    
    if ((axis_tilt >= 0) && (((unsigned int)axis_tilt) < joy_msg->get_axes_size()))
    {
      req_tilt += joy_msg->axes[axis_tilt] * tilt_step;
      req_tilt = std::max(std::min(req_tilt, max_tilt), -max_tilt);
    }
  }

  void send_cmd()
  {
    if (deadman_)
    { 
      sensor_msgs::JointState joint_cmds;
      joint_cmds.set_name_size(2);
      joint_cmds.set_position_size(2);
      joint_cmds.name[0] ="head_pan_joint";
      joint_cmds.position[0] = req_pan;
      joint_cmds.name[1] ="head_tilt_joint";
      joint_cmds.position[1] = req_tilt;
      head_pub_.publish(joint_cmds);
      fprintf(stdout,"teleop_head:: %f, %f\n", req_pan, req_tilt);  
    }
    else if (!deadman_no_publish_)
    {
      sensor_msgs::JointState joint_cmds;
      joint_cmds.set_name_size(2);
      joint_cmds.set_position_size(2);
      joint_cmds.name[0] ="head_pan_joint";
      joint_cmds.position[0] = req_pan;
      joint_cmds.name[1] ="head_tilt_joint";
      joint_cmds.position[1] = req_tilt;
      head_pub_.publish(joint_cmds);
      fprintf(stdout,"teleop_head:: %f, %f\n", req_pan, req_tilt);  
    }
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "teleop_head");
   const char* opt_no_publish    = "--deadman_no_publish";

   bool no_publish = false;
   for(int i=1;i<argc;i++)
   {
     if(!strncmp(argv[i], opt_no_publish, strlen(opt_no_publish)))
       no_publish = true;
   }

   TeleopHead teleop_head(no_publish);
   teleop_head.init();

   ros::Rate pub_rate(20);

   while (teleop_head.n_.ok())
   {
     ros::spinOnce(); 
     teleop_head.send_cmd();
     pub_rate.sleep();
   }
   
   exit(0);
   return 0;
}

