/*
 * teleop_head_keyboard
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

// Author: Kevin Watts

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>

#include <mechanism_msgs/JointState.h>
#include <mechanism_msgs/JointStates.h>

#define HEAD_TOPIC "/head_controller/command"

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77 

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57

class TeleopHeadKeyboard
{
  private:
  double req_tilt, req_pan;
  double max_pan, max_tilt, min_tilt, tilt_step, pan_step;

  ros::NodeHandle n_;
  ros::Publisher head_pub_;

  public:
  void init()
  {
    req_tilt = 0.0;
    req_pan = 0.0;
 
    head_pub_ = n_.advertise<mechanism_msgs::JointStates>(HEAD_TOPIC, 1);
    
    n_.param("max_pan", max_pan, 2.7);
    n_.param("max_tilt", max_tilt, 1.4);
    n_.param("min_tilt", min_tilt, -0.4);
    n_.param("tilt_step", tilt_step, 0.02);
    n_.param("pan_step", pan_step, 0.03);
    }
  
  ~TeleopHeadKeyboard()   { }
  void keyboardLoop();

};

//THK_Node* thk;
int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "head_keyboard");

  TeleopHeadKeyboard thk;
  thk.init();

  signal(SIGINT,quit);

  thk.keyboardLoop();

  return(0);
}

void TeleopHeadKeyboard::keyboardLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WASD' to pan and tilt");
  puts("Press 'Shift' to move faster");


  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    switch(c)
    {
    case KEYCODE_W:
      req_tilt += tilt_step;
      dirty = true;
      break;
    case KEYCODE_S:
      req_tilt -= tilt_step;
      dirty = true;
      break;
    case KEYCODE_A:
      req_pan += pan_step;
      dirty = true;
      break;
    case KEYCODE_D:
      req_pan -= pan_step;
      dirty = true;
      break;
    case KEYCODE_W_CAP:
      req_tilt += tilt_step * 3;
      dirty = true;
      break;
    case KEYCODE_S_CAP:
      req_tilt -= tilt_step * 3;
      dirty = true;
      break;
    case KEYCODE_A_CAP:
      req_pan += pan_step * 3;
      dirty = true;
      break;
    case KEYCODE_D_CAP:
      req_pan -= pan_step * 3;
      dirty = true;
      break;
    }

    req_tilt = std::max(std::min(req_tilt, max_tilt), min_tilt);
    req_pan = std::max(std::min(req_pan, max_pan), -max_pan);

    if (dirty == true)
    {
      
      mechanism_msgs::JointState joint_cmd ;
      mechanism_msgs::JointStates joint_cmds;
       
      joint_cmd.name ="head_pan_joint";
      joint_cmd.position = req_pan;
      joint_cmds.joints.push_back(joint_cmd);
      joint_cmd.name="head_tilt_joint";
      joint_cmd.position = req_tilt;
      joint_cmds.joints.push_back(joint_cmd);

      head_pub_.publish(joint_cmds) ;


    }
  }
}
