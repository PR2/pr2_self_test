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

/**

@mainpage

@b teleop_pr2_keyboard teleoperates a PR-2 base using keyboard commands. 
WASD controls X/Y, QE controls yaw. Shift to go faster.

<hr>

@section usage Usage
@verbatim
$ teleop_head_keyboard [standard ROS args]
@endverbatim

Key mappings are printed to screen on startup. 

<hr>

@section topic ROS topics

Subscribes to (name / type):
- None

Publishes to (name / type):
- @b "cmd_vel/PoseDot" : velocity to the pr2 base; sent on every keypress.

<hr>

@section parameters ROS parameters

- None

 **/

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <robot_msgs/PoseDot.h>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77 
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65

#define KEYCODE_A_CAP 0x41
#define KEYCODE_D_CAP 0x44
#define KEYCODE_S_CAP 0x53
#define KEYCODE_W_CAP 0x57
#define KEYCODE_Q_CAP 0x51
#define KEYCODE_E_CAP 0x45

class TeleopPR2Keyboard
{
  private:
  double walk_vel, run_vel, yaw_rate, yaw_rate_run;
  robot_msgs::PoseDot cmd;

  ros::NodeHandle n_;
  ros::Publisher vel_pub_;

  public:
  void init()
  { 
    cmd.vel.vx = cmd.vel.vy = cmd.ang_vel.vz = 0;

    vel_pub_ = n_.advertise<robot_msgs::PoseDot>("cmd_vel", 1);
    
    n_.param("walk_vel", walk_vel, 0.5);
    n_.param("run_vel", run_vel, 1.0);
    n_.param("yaw_rate", yaw_rate, 1.0);
    n_.param("yaw_run_rate", yaw_rate_run, 1.5);

  }
  
  ~TeleopPR2Keyboard()   { }
  void keyboardLoop();

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_base_keyboard");

  TeleopPR2Keyboard tpk;
  tpk.init();

  signal(SIGINT,quit);

  tpk.keyboardLoop();

  return(0);
}

void TeleopPR2Keyboard::keyboardLoop()
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
  puts("Use 'WASD' to translate");
  puts("Use 'QE' to yaw");
  puts("Press 'Shift' to run");


  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    cmd.vel.vx = cmd.vel.vy = cmd.ang_vel.vz = 0;

    switch(c)
    {
      // Walking
    case KEYCODE_W:
      cmd.vel.vx = walk_vel;
      dirty = true;
      break;
    case KEYCODE_S:
      cmd.vel.vx = - walk_vel;
      dirty = true;
      break;
    case KEYCODE_A:
      cmd.vel.vy = walk_vel;
      dirty = true;
      break;
    case KEYCODE_D:
      cmd.vel.vy = - walk_vel;
      dirty = true;
      break;
    case KEYCODE_Q:
      cmd.ang_vel.vz = yaw_rate;
      dirty = true;
      break;
    case KEYCODE_E:
      cmd.ang_vel.vz = - yaw_rate;
      dirty = true;
      break;

      // Running 
    case KEYCODE_W_CAP:
      cmd.vel.vx = run_vel;
      dirty = true;
      break;
    case KEYCODE_S_CAP:
      cmd.vel.vx = - run_vel;
      dirty = true;
      break;
    case KEYCODE_A_CAP:
      cmd.vel.vy = run_vel;
      dirty = true;
      break;
    case KEYCODE_D_CAP:
      cmd.vel.vy = - run_vel;
      dirty = true;
      break;
    case KEYCODE_Q_CAP:
      cmd.ang_vel.vz = yaw_rate_run;
      dirty = true;
      break;
    case KEYCODE_E_CAP:
      cmd.ang_vel.vz = - yaw_rate_run;
      dirty = true;
      break;
    }

    
    if (dirty == true)
    {
      vel_pub_.publish(cmd);
    }


  }
}
