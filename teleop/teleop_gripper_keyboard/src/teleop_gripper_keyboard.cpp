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
/*
 * Author: Sachin Chitta and Matthew Piccoli
 */

#include <ros/ros.h>
#include <ros/time.h>
#include <termios.h>
#include <signal.h>
#include <experimental_controllers/GripperControllerCmd.h>

//TODO::#ifndef PACKAGE_PATH_FILE_H me!
#define KEYCODE_0 0x30
#define KEYCODE_1 0x31
#define KEYCODE_2 0x32
#define KEYCODE_3 0x33
#define KEYCODE_4 0x34
#define KEYCODE_5 0x35
#define KEYCODE_6 0x36
#define KEYCODE_7 0x37
#define KEYCODE_8 0x38
#define KEYCODE_9 0x39
#define KEYCODE_S 0x53
#define KEYCODE_s 0x73
#define KEYCODE_R 0x52
#define KEYCODE_r 0x72
#define KEYCODE_L 0x4C
#define KEYCODE_l 0x6C
#define KEYCODE_C 0x43
#define KEYCODE_c 0x63
#define KEYCODE_m 0x6D
#define KEYCODE_M 0x4D
#define KEYCODE_o 0x6F
#define KEYCODE_O 0x4F
#define KEYCODE_T 0x54
#define KEYCODE_t 0x74
#define KEYCODE_H 0x48
#define KEYCODE_h 0x68
#define KEYCODE_B 0x42
#define KEYCODE_b 0x62
#define KEYCODE_G 0x47
#define KEYCODE_g 0x67
#define KEYCODE_SPACE 0x20
#define KEYCODE_PERIOD 0x2E
#define KEYCODE_MINUS 0x2D
#define KEYCODE_PLUS 0x2B
#define KEYCODE_EQUALS 0x3D
#define KEYCODE_UNDERSCORE 0x5F
#define KEYCODE_ENTER 0x0A
#define KEYCODE_RETURN 0x0D
#define KEYCODE_BACKSPACE 0x7F

class TGK_Node
{
  private:
    experimental_controllers::GripperControllerCmd cmd_val_;
    experimental_controllers::GripperControllerCmd cmd_evnt_;
    std::string topic_;
    double direction_;
    double val_mult_;

  ros::Publisher l_gripper_cmd_pub_, r_gripper_cmd_pub_;

  public:
  TGK_Node(ros::NodeHandle node)
    {
      val_mult_ = 1.0;
      direction_ = 1.0;
      cmd_val_.cmd = "move";
      cmd_evnt_.cmd = "event";
      topic_ = "pr2_gripper_controller/cmd";
      l_gripper_cmd_pub_ = node.advertise<experimental_controllers::GripperControllerCmd> ("l_gripper_cmd", 1);
      r_gripper_cmd_pub_ =  node.advertise<experimental_controllers::GripperControllerCmd> (topic_, 1);
    }
    ~TGK_Node()
    {
    }
    void keyboardLoop();
    void stopGripper();
};

TGK_Node* tgk;
int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tgk->stopGripper();
  
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "tgk");
  ros::NodeHandle n;
  tgk = new TGK_Node(n);

  signal(SIGINT, quit);

  tgk->keyboardLoop();

  return (0);
}

void TGK_Node::stopGripper()
{
  cmd_val_.cmd = "move";
  cmd_val_.val = 0.0;
  val_mult_ = 1.0;
  direction_ = 1.0;
  r_gripper_cmd_pub_.publish(cmd_val_);
}

void TGK_Node::keyboardLoop()
{
  char c;
  double temp_num;
  bool dirty = false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Space immediately stops the gripper at any time");
  puts("");
  puts("To switch hands type \"r\" or \"l\"");
  puts("");
  puts("To move: type \"m\" to enter move mode");
  puts("Set the direction with +,- (+ = open, - = close)");
  puts("Type a number to set the value (ex: 12.2)");
  puts("Press enter to execute or backspace to clear the number");
  puts("");
  puts("To step: type \"s\" to enter step mode");
  puts("Set the direction with +,- (+ = open, - = close)");
  puts("Type a number to set the step size (ex: 12.2)");
  puts("Press enter to execute or backspace to clear the number");
  puts("");
  puts("\"o\" opens immediately.");
  puts("\"c\" closes immediately.");
  puts("");
  puts("\"t\" sends a touching event");
  puts("\"h\" sends a holding event");
  puts("\"b\" sends a breaking (crushing) event");
  puts("");
  puts("anything else also stops the gripper");
  puts("---------------------------");

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
      case KEYCODE_T: //touching object, but not holding it yet!
      case KEYCODE_t:
        cmd_val_.cmd = "moveTo";
        break;
      case KEYCODE_H: //holding object, but not crushing it yet!
      case KEYCODE_h:
        cmd_evnt_.val = 2.0;
        cmd_evnt_.time = ros::Time::now().toSec();
        r_gripper_cmd_pub_.publish(cmd_evnt_);
        break;
      case KEYCODE_B: //breaking (crushing) object!
      case KEYCODE_b:
        cmd_evnt_.val = 3.0;
        cmd_evnt_.time = ros::Time::now().toSec();
        r_gripper_cmd_pub_.publish(cmd_evnt_);
        break;
      case KEYCODE_M:
      case KEYCODE_m:
        cmd_val_.cmd = "move";
        break;
      case KEYCODE_S:
      case KEYCODE_s:
        cmd_val_.cmd = "step";
        break;
      case KEYCODE_O:
      case KEYCODE_o:
        cmd_val_.cmd = "open";
        dirty = true;
        break;
      case KEYCODE_C:
      case KEYCODE_c:
        cmd_val_.cmd = "close";
        dirty = true;
        break;
      case KEYCODE_G:
      case KEYCODE_g:
        cmd_val_.cmd = "grasp";
        break;
      case KEYCODE_L:
      case KEYCODE_l:
        topic_ = "l_gripper_cmd";
        break;
      case KEYCODE_R:
      case KEYCODE_r:
        topic_ = "r_gripper_cmd";
        break;
      case KEYCODE_BACKSPACE:
        cmd_val_.val = 0.0;
        val_mult_ = 1.0;
        direction_ = 1.0;
        break;
      case KEYCODE_0:
        temp_num = 0.0;
        if(val_mult_ == 1.0)
          cmd_val_.val = cmd_val_.val * 10.0 + temp_num * direction_;
        else
        {
          cmd_val_.val += val_mult_ * temp_num * direction_;
          val_mult_ = val_mult_ / 10.0;
        }
        break;
      case KEYCODE_1:
        temp_num = 1.0;
        if(val_mult_ == 1.0)
          cmd_val_.val = cmd_val_.val * 10.0 + temp_num * direction_;
        else
        {
          cmd_val_.val += val_mult_ * temp_num * direction_;
          val_mult_ = val_mult_ / 10.0;
        }
        break;
      case KEYCODE_2:
        temp_num = 2.0;
        if(val_mult_ == 1.0)
          cmd_val_.val = cmd_val_.val * 10.0 + temp_num * direction_;
        else
        {
          cmd_val_.val += val_mult_ * temp_num * direction_;
          val_mult_ = val_mult_ / 10.0;
        }
        break;
      case KEYCODE_3:
        temp_num = 3.0;
        if(val_mult_ == 1.0)
          cmd_val_.val = cmd_val_.val * 10.0 + temp_num * direction_;
        else
        {
          cmd_val_.val += val_mult_ * temp_num * direction_;
          val_mult_ = val_mult_ / 10.0;
        }
        break;
      case KEYCODE_4:
        temp_num = 4.0;
        if(val_mult_ == 1.0)
          cmd_val_.val = cmd_val_.val * 10.0 + temp_num * direction_;
        else
        {
          cmd_val_.val += val_mult_ * temp_num * direction_;
          val_mult_ = val_mult_ / 10.0;
        }
        break;
      case KEYCODE_5:
        temp_num = 5.0;
        if(val_mult_ == 1.0)
          cmd_val_.val = cmd_val_.val * 10.0 + temp_num * direction_;
        else
        {
          cmd_val_.val += val_mult_ * temp_num * direction_;
          val_mult_ = val_mult_ / 10.0;
        }
        break;
      case KEYCODE_6:
        temp_num = 6.0;
        if(val_mult_ == 1.0)
          cmd_val_.val = cmd_val_.val * 10.0 + temp_num * direction_;
        else
        {
          cmd_val_.val += val_mult_ * temp_num * direction_;
          val_mult_ = val_mult_ / 10.0;
        }
        break;
      case KEYCODE_7:
        temp_num = 7.0;
        if(val_mult_ == 1.0)
          cmd_val_.val = cmd_val_.val * 10.0 + temp_num * direction_;
        else
        {
          cmd_val_.val += val_mult_ * temp_num * direction_;
          val_mult_ = val_mult_ / 10.0;
        }
        break;
      case KEYCODE_8:
        temp_num = 8.0;
        if(val_mult_ == 1.0)
          cmd_val_.val = cmd_val_.val * 10.0 + temp_num * direction_;
        else
        {
          cmd_val_.val += val_mult_ * temp_num * direction_;
          val_mult_ = val_mult_ / 10.0;
        }
        break;
      case KEYCODE_9:
        temp_num = 9.0;
        if(val_mult_ == 1.0)
          cmd_val_.val = cmd_val_.val * 10.0 + temp_num * direction_;
        else
        {
          cmd_val_.val += val_mult_ * temp_num * direction_;
          val_mult_ = val_mult_ / 10.0;
        }
        break;
      case KEYCODE_PERIOD:
        val_mult_ = 0.1;
        break;
      case KEYCODE_UNDERSCORE:
      case KEYCODE_MINUS:
        if(direction_ != -1.0)
        {
          direction_ = -1.0;
          cmd_val_.val *= -1.0;
        }
        break;
      case KEYCODE_EQUALS:
      case KEYCODE_PLUS:
        if(direction_ != 1.0)
        {
          direction_ = 1.0;
          cmd_val_.val *= -1.0;
        }
        break;
      case KEYCODE_RETURN:
      case KEYCODE_ENTER:
        dirty = true;
        break;
      case KEYCODE_SPACE:
        stopGripper();
        break;
      default:
        ROS_INFO("Char %c (%i) is not known", c, c);
        stopGripper();
    }
    //printf("%s %s %f\n", topic_.c_str(), cmd_val_.cmd.c_str(), cmd_val_.val);
    if(dirty == true)
    {
      r_gripper_cmd_pub_.publish(cmd_val_);
      dirty = false;
    }
  }
}
