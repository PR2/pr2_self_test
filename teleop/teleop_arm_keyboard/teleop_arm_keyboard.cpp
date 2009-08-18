/*
 * Copyright (c) 2008, Willow Garage, Inc.  All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.  * Redistributions
 *     in binary form must reproduce the above copyright notice, this list of
 *     conditions and the following disclaimer in the documentation and/or
 *     other materials provided with the distribution.  * Neither the name of
 *     the Willow Garage, Inc. nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without specific
 *     prior written permission.
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

// Author: Advait Jain, John Hsu

/**

  @mainpage

  @htmlinclude manifest.html

  @b teleop_arm_keyboard teleoperates the arms of the PR2 by mapping
  key presses into joint angle set points.

  <hr>

  @section usage Usage
  @verbatim
  $ teleop_arm_keyboard [standard ROS args]
  @endverbatim

  Key mappings are printed to screen on startup.

  <hr>

  @section topic ROS topics

  Subscribes to (name/type):
  - None

  Publishes to (name / type):
  - @b "lArmCmd"/JointPosCmd : configuration of the left arm (all the joint angles); sent on every keypress.

  <hr>

  @section parameters ROS parameters

  - None

 **/

#include <termios.h>
#include <signal.h>
#include <math.h>

#include <ros/node.h>
#include <pr2_mechanism_controllers/JointPosCmd.h>
#include <std_msgs/Float64.h>

#define COMMAND_TIMEOUT_SEC 0.2

#define L_CONTROLLER_NAME_0 "l_shoulder_pan_position_controller"
#define L_CONTROLLER_NAME_1 "l_shoulder_lift_position_controller"
#define L_CONTROLLER_NAME_2 "l_upper_arm_roll_position_controller"
#define L_CONTROLLER_NAME_3 "l_elbow_flex_position_controller"
#define L_CONTROLLER_NAME_4 "l_forearm_roll_position_controller"
#define L_CONTROLLER_NAME_5 "l_wrist_flex_position_controller"
#define L_CONTROLLER_NAME_6 "l_wrist_roll_position_controller"
#define L_CONTROLLER_NAME_7 "l_gripper_position_controller"
#define R_CONTROLLER_NAME_0 "r_shoulder_pan_position_controller"
#define R_CONTROLLER_NAME_1 "r_shoulder_lift_position_controller"
#define R_CONTROLLER_NAME_2 "r_upper_arm_roll_position_controller"
#define R_CONTROLLER_NAME_3 "r_elbow_flex_position_controller"
#define R_CONTROLLER_NAME_4 "r_forearm_roll_position_controller"
#define R_CONTROLLER_NAME_5 "r_wrist_flex_position_controller"
#define R_CONTROLLER_NAME_6 "r_wrist_roll_position_controller"
#define R_CONTROLLER_NAME_7 "r_gripper_position_controller"

#define L_COMMAND_TOPIC_0 L_CONTROLLER_NAME_0"/set_command"
#define L_COMMAND_TOPIC_1 L_CONTROLLER_NAME_1"/set_command"
#define L_COMMAND_TOPIC_2 L_CONTROLLER_NAME_2"/set_command"
#define L_COMMAND_TOPIC_3 L_CONTROLLER_NAME_3"/set_command"
#define L_COMMAND_TOPIC_4 L_CONTROLLER_NAME_4"/set_command"
#define L_COMMAND_TOPIC_5 L_CONTROLLER_NAME_5"/set_command"
#define L_COMMAND_TOPIC_6 L_CONTROLLER_NAME_6"/set_command"
#define L_COMMAND_TOPIC_7 L_CONTROLLER_NAME_7"/set_command"
#define R_COMMAND_TOPIC_0 R_CONTROLLER_NAME_0"/set_command"
#define R_COMMAND_TOPIC_1 R_CONTROLLER_NAME_1"/set_command"
#define R_COMMAND_TOPIC_2 R_CONTROLLER_NAME_2"/set_command"
#define R_COMMAND_TOPIC_3 R_CONTROLLER_NAME_3"/set_command"
#define R_COMMAND_TOPIC_4 R_CONTROLLER_NAME_4"/set_command"
#define R_COMMAND_TOPIC_5 R_CONTROLLER_NAME_5"/set_command"
#define R_COMMAND_TOPIC_6 R_CONTROLLER_NAME_6"/set_command"
#define R_COMMAND_TOPIC_7 R_CONTROLLER_NAME_7"/set_command"

#define L_STEP_SIZE_0 1.0*M_PI/180.0
#define L_STEP_SIZE_1 1.0*M_PI/180.0
#define L_STEP_SIZE_2 1.0*M_PI/180.0
#define L_STEP_SIZE_3 1.0*M_PI/180.0
#define L_STEP_SIZE_4 1.0*M_PI/180.0
#define L_STEP_SIZE_5 1.0*M_PI/180.0
#define L_STEP_SIZE_6 1.0*M_PI/180.0
#define L_STEP_SIZE_7 1.0*M_PI/180.0
#define R_STEP_SIZE_0 1.0*M_PI/180.0
#define R_STEP_SIZE_1 1.0*M_PI/180.0
#define R_STEP_SIZE_2 1.0*M_PI/180.0
#define R_STEP_SIZE_3 1.0*M_PI/180.0
#define R_STEP_SIZE_4 1.0*M_PI/180.0
#define R_STEP_SIZE_5 1.0*M_PI/180.0
#define R_STEP_SIZE_6 1.0*M_PI/180.0
#define R_STEP_SIZE_7 1.0*M_PI/180.0

/// @todo Remove this giant enum, which was stoled from pr2Core/pr2Core.h.
/// It can be replaced by some simpler indexing scheme.

class TeleopArmKeyboardNode : public ros::Node
{
  private:
    std_msgs::Float64 lArmCmd[8];
    std_msgs::Float64 rArmCmd[8];

  public:
    double lArmCmdStep[8];
    double rArmCmdStep[8];
    // constructor
    TeleopArmKeyboardNode() : ros::Node("teleop_arm_keyboard_node")
    {

      // initialize step size
      this->lArmCmdStep[0] = L_STEP_SIZE_0;
      this->lArmCmdStep[1] = L_STEP_SIZE_1;
      this->lArmCmdStep[2] = L_STEP_SIZE_2;
      this->lArmCmdStep[3] = L_STEP_SIZE_3;
      this->lArmCmdStep[4] = L_STEP_SIZE_4;
      this->lArmCmdStep[5] = L_STEP_SIZE_5;
      this->lArmCmdStep[6] = L_STEP_SIZE_6;
      this->lArmCmdStep[7] = L_STEP_SIZE_7;

      this->rArmCmdStep[0] = R_STEP_SIZE_0;
      this->rArmCmdStep[1] = R_STEP_SIZE_1;
      this->rArmCmdStep[2] = R_STEP_SIZE_2;
      this->rArmCmdStep[3] = R_STEP_SIZE_3;
      this->rArmCmdStep[4] = R_STEP_SIZE_4;
      this->rArmCmdStep[5] = R_STEP_SIZE_5;
      this->rArmCmdStep[6] = R_STEP_SIZE_6;
      this->rArmCmdStep[7] = R_STEP_SIZE_7;

      // advertise
      advertise<std_msgs::Float64>(L_COMMAND_TOPIC_0,1);
      advertise<std_msgs::Float64>(L_COMMAND_TOPIC_1,1);
      advertise<std_msgs::Float64>(L_COMMAND_TOPIC_2,1);
      advertise<std_msgs::Float64>(L_COMMAND_TOPIC_3,1);
      advertise<std_msgs::Float64>(L_COMMAND_TOPIC_4,1);
      advertise<std_msgs::Float64>(L_COMMAND_TOPIC_5,1);
      advertise<std_msgs::Float64>(L_COMMAND_TOPIC_6,1);
      advertise<std_msgs::Float64>(L_COMMAND_TOPIC_7,1);
      advertise<std_msgs::Float64>(R_COMMAND_TOPIC_0,1);
      advertise<std_msgs::Float64>(R_COMMAND_TOPIC_1,1);
      advertise<std_msgs::Float64>(R_COMMAND_TOPIC_2,1);
      advertise<std_msgs::Float64>(R_COMMAND_TOPIC_3,1);
      advertise<std_msgs::Float64>(R_COMMAND_TOPIC_4,1);
      advertise<std_msgs::Float64>(R_COMMAND_TOPIC_5,1);
      advertise<std_msgs::Float64>(R_COMMAND_TOPIC_6,1);
      advertise<std_msgs::Float64>(R_COMMAND_TOPIC_7,1);

      // cmd_armconfig should probably be initialised
      // with the current joint angles of the arm rather
      // than zeros.
      this->lArmCmd[0].data     = 0;
      this->lArmCmd[1].data     = 0;
      this->lArmCmd[2].data     = 0;
      this->lArmCmd[3].data     = 0;
      this->lArmCmd[4].data     = 0;
      this->lArmCmd[5].data     = 0;
      this->lArmCmd[6].data     = 0;
      this->lArmCmd[7].data     = 0;
      this->rArmCmd[0].data     = 0;
      this->rArmCmd[1].data     = 0;
      this->rArmCmd[2].data     = 0;
      this->rArmCmd[3].data     = 0;
      this->rArmCmd[4].data     = 0;
      this->rArmCmd[5].data     = 0;
      this->rArmCmd[6].data     = 0;
      this->rArmCmd[7].data     = 0;

    }
    // destructor
    ~TeleopArmKeyboardNode() { }

    void printCurrentJointValues() {
      std::cout << "left arm " << std::endl;
      std::cout << " cmds: "
          << " " << this->lArmCmd[0].data
          << " " << this->lArmCmd[1].data
          << " " << this->lArmCmd[2].data
          << " " << this->lArmCmd[3].data
          << " " << this->lArmCmd[4].data
          << " " << this->lArmCmd[5].data
          << " " << this->lArmCmd[6].data
          << " " << this->lArmCmd[7].data
          << std::endl;
      std::cout << "right arm " << std::endl;
      std::cout << " cmds: "
          << " " << this->rArmCmd[0].data
          << " " << this->rArmCmd[1].data
          << " " << this->rArmCmd[2].data
          << " " << this->rArmCmd[3].data
          << " " << this->rArmCmd[4].data
          << " " << this->rArmCmd[5].data
          << " " << this->rArmCmd[6].data
          << " " << this->rArmCmd[7].data
          << std::endl;

    }

    void keyboardLoop();
    void changeJointAngle(std::string joint_name, double direction);
    void changeJointStepSize(std::string joint_name, double direction);
    void openGripper(std::string joint_name);
    void closeGripper(std::string joint_name);

};

// Stuff for keyboard interaction
int kfd = 0;
struct termios cooked, raw;
void quit(int sig)
{
  //  tbk->stopRobot();
  
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}



int main(int argc, char** argv)
{
  ros::init(argc,argv);

  TeleopArmKeyboardNode* takn = new TeleopArmKeyboardNode();

  signal(SIGINT,quit);

  takn->keyboardLoop();

  return(0);
}

void TeleopArmKeyboardNode::openGripper(std::string joint_name) {
  if(joint_name == R_CONTROLLER_NAME_7 ) {
    this->rArmCmd[7].data = .06;
    printf("Opening right gripper\n");
  } else if(joint_name == L_CONTROLLER_NAME_7 ) {
    this->lArmCmd[7].data = .06;
    printf("Opening left gripper\n");
  }
}

void TeleopArmKeyboardNode::closeGripper(std::string joint_name) {
  if(joint_name == R_CONTROLLER_NAME_7 ) {
    this->rArmCmd[7].data = .0;
  } else if(joint_name == L_CONTROLLER_NAME_7 ) {
    this->lArmCmd[7].data = .0;
  }
}


void TeleopArmKeyboardNode::changeJointAngle(std::string joint_name, double direction)
{
  if (joint_name == L_CONTROLLER_NAME_0)
      this->lArmCmd[0].data += direction * this->lArmCmdStep[0];
  if (joint_name == L_CONTROLLER_NAME_1)
      this->lArmCmd[1].data += direction * this->lArmCmdStep[1];
  if (joint_name == L_CONTROLLER_NAME_2)
      this->lArmCmd[2].data += direction * this->lArmCmdStep[2];
  if (joint_name == L_CONTROLLER_NAME_3)
      this->lArmCmd[3].data += direction * this->lArmCmdStep[3];
  if (joint_name == L_CONTROLLER_NAME_4)
      this->lArmCmd[4].data += direction * this->lArmCmdStep[4];
  if (joint_name == L_CONTROLLER_NAME_5)
      this->lArmCmd[5].data += direction * this->lArmCmdStep[5];
  if (joint_name == L_CONTROLLER_NAME_6)
      this->lArmCmd[6].data += direction * this->lArmCmdStep[6];
  if (joint_name == L_CONTROLLER_NAME_7)
      this->lArmCmd[7].data += direction * this->lArmCmdStep[7];

  if (joint_name == R_CONTROLLER_NAME_0)
      this->rArmCmd[0].data += direction * this->rArmCmdStep[0];
  if (joint_name == R_CONTROLLER_NAME_1)
      this->rArmCmd[1].data += direction * this->rArmCmdStep[1];
  if (joint_name == R_CONTROLLER_NAME_2)
      this->rArmCmd[2].data += direction * this->rArmCmdStep[2];
  if (joint_name == R_CONTROLLER_NAME_3)
      this->rArmCmd[3].data += direction * this->rArmCmdStep[3];
  if (joint_name == R_CONTROLLER_NAME_4)
      this->rArmCmd[4].data += direction * this->rArmCmdStep[4];
  if (joint_name == R_CONTROLLER_NAME_5)
      this->rArmCmd[5].data += direction * this->rArmCmdStep[5];
  if (joint_name == R_CONTROLLER_NAME_6)
      this->rArmCmd[6].data += direction * this->rArmCmdStep[6];
  if (joint_name == R_CONTROLLER_NAME_7)
      this->rArmCmd[7].data += direction * this->rArmCmdStep[7];
}

void TeleopArmKeyboardNode::changeJointStepSize(std::string joint_name, double direction)
{
  if (joint_name == L_CONTROLLER_NAME_0)
      this->lArmCmdStep[0] *= direction;
  if (joint_name == L_CONTROLLER_NAME_1)
      this->lArmCmdStep[1] *= direction;
  if (joint_name == L_CONTROLLER_NAME_2)
      this->lArmCmdStep[2] *= direction;
  if (joint_name == L_CONTROLLER_NAME_3)
      this->lArmCmdStep[3] *= direction;
  if (joint_name == L_CONTROLLER_NAME_4)
      this->lArmCmdStep[4] *= direction;
  if (joint_name == L_CONTROLLER_NAME_5)
      this->lArmCmdStep[5] *= direction;
  if (joint_name == L_CONTROLLER_NAME_6)
      this->lArmCmdStep[6] *= direction;
  if (joint_name == L_CONTROLLER_NAME_7)
      this->lArmCmdStep[7] *= direction;

  if (joint_name == R_CONTROLLER_NAME_0)
      this->rArmCmdStep[0] *= direction;
  if (joint_name == R_CONTROLLER_NAME_1)
      this->rArmCmdStep[1] *= direction;
  if (joint_name == R_CONTROLLER_NAME_2)
      this->rArmCmdStep[2] *= direction;
  if (joint_name == R_CONTROLLER_NAME_3)
      this->rArmCmdStep[3] *= direction;
  if (joint_name == R_CONTROLLER_NAME_4)
      this->rArmCmdStep[4] *= direction;
  if (joint_name == R_CONTROLLER_NAME_5)
      this->rArmCmdStep[5] *= direction;
  if (joint_name == R_CONTROLLER_NAME_6)
      this->rArmCmdStep[6] *= direction;
  if (joint_name == R_CONTROLLER_NAME_7)
      this->rArmCmdStep[7] *= direction;
}



void TeleopArmKeyboardNode::keyboardLoop()
{
  char c;
  bool dirty=false;
  std::string current_joint_name = L_CONTROLLER_NAME_0; // joint which will be actuated.
  bool right_arm = false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  printf("Press l/r to operate left/right arm.\n");
  printf("Numbers 1 through 8 to select the joint to operate.\n");
  printf("9 initializes teleop_arm_keyboard's robot state with the state of the robot in the simulation.\n");
  printf("+ and - will move the joint in different directions by 5 degrees.\n");
  puts("");
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
      case 'l':
      case 'L':
        right_arm = false;
        printf("Actuating left arm.\n");
        break;
      case 'r':
      case 'R':
        right_arm = true;
        printf("Actuating right arm.\n");
        break;
      case 'a':
      case 'A':
        changeJointStepSize(current_joint_name,2.0);
        printf("increase step size.\n");
        break;
      case 'z':
      case 'Z':
        changeJointStepSize(current_joint_name,0.5);
        printf("decrease step size.\n");
        break;
      case '+':
      case '=':
        changeJointAngle(current_joint_name, 1);
        dirty=true;
        break;
      case '_':
      case '-':
        changeJointAngle(current_joint_name, -1);
        dirty=true;
        break;
      case '.':
        openGripper(current_joint_name);
        dirty = true;
        break;
      case '/':
        sleep(1);
        closeGripper(current_joint_name);
        dirty = true;
        break;
      case 'q':
        printCurrentJointValues();
        break;
      default:
        break;
    }

    if (right_arm==false)
    {
      switch(c)
      {
        case '1':
          current_joint_name = L_CONTROLLER_NAME_0;
          printf("left turret\n");
          break;
        case '2':
          current_joint_name = L_CONTROLLER_NAME_1;
          printf("left shoulder pitch\n");
          break;
        case '3':
          current_joint_name = L_CONTROLLER_NAME_2;
          printf("left shoulder roll\n");
          break;
        case '4':
          current_joint_name = L_CONTROLLER_NAME_3;
          printf("left elbow pitch\n");
          break;
        case '5':
          current_joint_name = L_CONTROLLER_NAME_4;
          printf("left elbow roll\n");
          break;
        case '6':
          current_joint_name = L_CONTROLLER_NAME_5;
          printf("left wrist pitch\n");
          break;
        case '7':
          current_joint_name = L_CONTROLLER_NAME_6;
          printf("left wrist roll\n");
          break;
        case '8':
          current_joint_name = L_CONTROLLER_NAME_7;
          printf("left gripper\n");
          break;
        case '9':
          printf("Resetting left commands to current position.\n");
        default:
          break;
      }
    }
    else
    {
      switch(c)
      {
        case '1':
          current_joint_name = R_CONTROLLER_NAME_0;
          printf("right turret\n");
          break;
        case '2':
          current_joint_name = R_CONTROLLER_NAME_1;
          printf("right shoulder pitch\n");
          break;
        case '3':
          current_joint_name = R_CONTROLLER_NAME_2;
          printf("right shoulder roll\n");
          break;
        case '4':
          current_joint_name = R_CONTROLLER_NAME_3;
          printf("right elbow pitch\n");
          break;
        case '5':
          current_joint_name = R_CONTROLLER_NAME_4;
          printf("right elbow roll\n");
          break;
        case '6':
          current_joint_name = R_CONTROLLER_NAME_5;
          printf("right wrist pitch\n");
          break;
        case '7':
          current_joint_name = R_CONTROLLER_NAME_6;
          printf("right wrist roll\n");
          break;
        case '8':
          current_joint_name = R_CONTROLLER_NAME_7;
          printf("right gripper\n");
          break;
        case '9':
          printf("Resetting right commands to current position.\n");
        default:
          break;
      }
    }

    if(!right_arm) {
      publish(L_COMMAND_TOPIC_0,lArmCmd[0]);
      publish(L_COMMAND_TOPIC_1,lArmCmd[1]);
      publish(L_COMMAND_TOPIC_2,lArmCmd[2]);
      publish(L_COMMAND_TOPIC_3,lArmCmd[3]);
      publish(L_COMMAND_TOPIC_4,lArmCmd[4]);
      publish(L_COMMAND_TOPIC_5,lArmCmd[5]);
      publish(L_COMMAND_TOPIC_6,lArmCmd[6]);
      publish(L_COMMAND_TOPIC_7,lArmCmd[7]);
    } else {
      publish(R_COMMAND_TOPIC_0,rArmCmd[0]);
      publish(R_COMMAND_TOPIC_1,rArmCmd[1]);
      publish(R_COMMAND_TOPIC_2,rArmCmd[2]);
      publish(R_COMMAND_TOPIC_3,rArmCmd[3]);
      publish(R_COMMAND_TOPIC_4,rArmCmd[4]);
      publish(R_COMMAND_TOPIC_5,rArmCmd[5]);
      publish(R_COMMAND_TOPIC_6,rArmCmd[6]);
      publish(R_COMMAND_TOPIC_7,rArmCmd[7]);
    }
  }
}


