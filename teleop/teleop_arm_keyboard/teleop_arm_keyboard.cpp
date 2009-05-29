/*
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
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
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

#define L_LINK_NAME_0 "l_shoulder_pan_joint"
#define L_LINK_NAME_1 "l_shoulder_lift_joint"
#define L_LINK_NAME_2 "l_upper_arm_roll_joint"
#define L_LINK_NAME_3 "l_elbow_flex_joint"
#define L_LINK_NAME_4 "l_forearm_roll_joint"
#define L_LINK_NAME_5 "l_wrist_flex_joint"
#define L_LINK_NAME_6 "l_wrist_roll_joint"
#define L_LINK_NAME_7 "l_gripper_joint"
#define R_LINK_NAME_0 "r_shoulder_pan_joint"
#define R_LINK_NAME_1 "r_shoulder_lift_joint"
#define R_LINK_NAME_2 "r_upper_arm_roll_joint"
#define R_LINK_NAME_3 "r_elbow_flex_joint"
#define R_LINK_NAME_4 "r_forearm_roll_joint"
#define R_LINK_NAME_5 "r_wrist_flex_joint"
#define R_LINK_NAME_6 "r_wrist_roll_joint"
#define R_LINK_NAME_7 "r_gripper_joint"

#define LEFT_ARM_COMMAND_TOPIC "left_arm_commands"
#define RIGHT_ARM_COMMAND_TOPIC "right_arm_commands"
#define LEFT_GRIPPER_COMMAND_TOPIC "l_gripper_controller/set_command"
#define RIGHT_GRIPPER_COMMAND_TOPIC "r_gripper_controller/set_command"
#define JOINT_STEP_SIZE   5*M_PI/180
#define GRIPPER_STEP_SIZE 1*M_PI/180


/// @todo Remove this giant enum, which was stoled from pr2Core/pr2Core.h.
/// It can be replaced by some simpler indexing scheme.

class TeleopArmKeyboardNode : public ros::Node
{
  private:
    pr2_mechanism_controllers::JointPosCmd lArmCmd;
    pr2_mechanism_controllers::JointPosCmd rArmCmd;
    std_msgs::Float64 lGripperCmd;
    std_msgs::Float64 rGripperCmd;

  public:
    double jointCmdStep;
    double gripperStep;
    // constructor
    TeleopArmKeyboardNode() : ros::Node("teleop_arm_keyboard_node")
    {

      // initialize step size
      this->jointCmdStep = JOINT_STEP_SIZE;
      this->gripperStep  = GRIPPER_STEP_SIZE;

      // cmd_armconfig should probably be initialised
      // with the current joint angles of the arm rather
      // than zeros.
      this->lArmCmd.set_names_size(7);
      this->rArmCmd.set_names_size(7);
      this->lArmCmd.set_positions_size(7);
      this->rArmCmd.set_positions_size(7);
      this->lArmCmd.set_margins_size(7);
      this->rArmCmd.set_margins_size(7);

      this->lArmCmd.names[0] = L_LINK_NAME_0;
      this->lArmCmd.names[1] = L_LINK_NAME_1;
      this->lArmCmd.names[2] = L_LINK_NAME_2;
      this->lArmCmd.names[3] = L_LINK_NAME_3;
      this->lArmCmd.names[4] = L_LINK_NAME_4;
      this->lArmCmd.names[5] = L_LINK_NAME_5;
      this->lArmCmd.names[6] = L_LINK_NAME_6;

      this->lArmCmd.positions[0] = 0;
      this->lArmCmd.positions[1] = 0;
      this->lArmCmd.positions[2] = 0;
      this->lArmCmd.positions[3] = 0;
      this->lArmCmd.positions[4] = 0;
      this->lArmCmd.positions[5] = 0;
      this->lArmCmd.positions[6] = 0;

      this->lArmCmd.margins[0] = 0;
      this->lArmCmd.margins[1] = 0;
      this->lArmCmd.margins[2] = 0;
      this->lArmCmd.margins[3] = 0;
      this->lArmCmd.margins[4] = 0;
      this->lArmCmd.margins[5] = 0;
      this->lArmCmd.margins[6] = 0;

      this->rArmCmd.names[0] = R_LINK_NAME_0;
      this->rArmCmd.names[1] = R_LINK_NAME_1;
      this->rArmCmd.names[2] = R_LINK_NAME_2;
      this->rArmCmd.names[3] = R_LINK_NAME_3;
      this->rArmCmd.names[4] = R_LINK_NAME_4;
      this->rArmCmd.names[5] = R_LINK_NAME_5;
      this->rArmCmd.names[6] = R_LINK_NAME_6;

      this->rArmCmd.positions[0] = 0;
      this->rArmCmd.positions[1] = 0;
      this->rArmCmd.positions[2] = 0;
      this->rArmCmd.positions[3] = 0;
      this->rArmCmd.positions[4] = 0;
      this->rArmCmd.positions[5] = 0;
      this->rArmCmd.positions[6] = 0;

      this->rArmCmd.margins[0] = 0;
      this->rArmCmd.margins[1] = 0;
      this->rArmCmd.margins[2] = 0;
      this->rArmCmd.margins[3] = 0;
      this->rArmCmd.margins[4] = 0;
      this->rArmCmd.margins[5] = 0;
      this->rArmCmd.margins[6] = 0;

      advertise<pr2_mechanism_controllers::JointPosCmd>(LEFT_ARM_COMMAND_TOPIC,1);
      advertise<pr2_mechanism_controllers::JointPosCmd>(RIGHT_ARM_COMMAND_TOPIC,1);
      advertise<std_msgs::Float64>(LEFT_GRIPPER_COMMAND_TOPIC,1);
      advertise<std_msgs::Float64>(RIGHT_GRIPPER_COMMAND_TOPIC,1);

      // deal with grippers separately
      this->lGripperCmd.data     = 0;
      this->rGripperCmd.data     = 0;

    }
    // destructor
    ~TeleopArmKeyboardNode() { }

    void printCurrentJointValues() {
      std::cout << "left arm " << std::endl;
      std::cout << " cmds: "
          << " " << this->lArmCmd.positions[0]
          << " " << this->lArmCmd.positions[1]
          << " " << this->lArmCmd.positions[2]
          << " " << this->lArmCmd.positions[3]
          << " " << this->lArmCmd.positions[4]
          << " " << this->lArmCmd.positions[5]
          << " " << this->lArmCmd.positions[6]
          << " " << this->lGripperCmd.data
          << std::endl;
      std::cout << "right arm " << std::endl;
      std::cout << " cmds: "
          << " " << this->rArmCmd.positions[0]
          << " " << this->rArmCmd.positions[1]
          << " " << this->rArmCmd.positions[2]
          << " " << this->rArmCmd.positions[3]
          << " " << this->rArmCmd.positions[4]
          << " " << this->rArmCmd.positions[5]
          << " " << this->rArmCmd.positions[6]
          << " " << this->rGripperCmd.data
          << std::endl;

    }

    void keyboardLoop();
    void changeJointAngle(std::string joint_name, double direction);
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
  if(joint_name == R_LINK_NAME_7 ) {
    this->rGripperCmd.data = .2;
    printf("Opening right gripper\n");
  } else if(joint_name == L_LINK_NAME_7 ) {
    this->lGripperCmd.data = .2;
    printf("Opening left gripper\n");
  }
}

void TeleopArmKeyboardNode::closeGripper(std::string joint_name) {
  if(joint_name == R_LINK_NAME_7 ) {
    this->rGripperCmd.data = 0;
  } else if(joint_name == L_LINK_NAME_7 ) {
    this->lGripperCmd.data = 0;
  }
}


void TeleopArmKeyboardNode::changeJointAngle(std::string joint_name, double direction)
{
  if (joint_name == L_LINK_NAME_0)
      this->lArmCmd.positions[0] += direction * this->jointCmdStep;
  if (joint_name == L_LINK_NAME_1)
      this->lArmCmd.positions[1] += direction * this->jointCmdStep;
  if (joint_name == L_LINK_NAME_2)
      this->lArmCmd.positions[2] += direction * this->jointCmdStep;
  if (joint_name == L_LINK_NAME_3)
      this->lArmCmd.positions[3] += direction * this->jointCmdStep;
  if (joint_name == L_LINK_NAME_4)
      this->lArmCmd.positions[4] += direction * this->jointCmdStep;
  if (joint_name == L_LINK_NAME_5)
      this->lArmCmd.positions[5] += direction * this->jointCmdStep;
  if (joint_name == L_LINK_NAME_6)
      this->lArmCmd.positions[6] += direction * this->jointCmdStep;
  if (joint_name == L_LINK_NAME_7)
      this->lGripperCmd.data     += direction * this->gripperStep;

  if (joint_name == R_LINK_NAME_0)
      this->rArmCmd.positions[0] += direction * this->jointCmdStep;
  if (joint_name == R_LINK_NAME_1)
      this->rArmCmd.positions[1] += direction * this->jointCmdStep;
  if (joint_name == R_LINK_NAME_2)
      this->rArmCmd.positions[2] += direction * this->jointCmdStep;
  if (joint_name == R_LINK_NAME_3)
      this->rArmCmd.positions[3] += direction * this->jointCmdStep;
  if (joint_name == R_LINK_NAME_4)
      this->rArmCmd.positions[4] += direction * this->jointCmdStep;
  if (joint_name == R_LINK_NAME_5)
      this->rArmCmd.positions[5] += direction * this->jointCmdStep;
  if (joint_name == R_LINK_NAME_6)
      this->rArmCmd.positions[6] += direction * this->jointCmdStep;
  if (joint_name == R_LINK_NAME_7)
      this->rGripperCmd.data     += direction * this->gripperStep;
}


void TeleopArmKeyboardNode::keyboardLoop()
{
  char c;
  bool dirty=false;
  std::string current_joint_name = L_LINK_NAME_0; // joint which will be actuated.
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
          current_joint_name = L_LINK_NAME_0;
          printf("left turret\n");
          break;
        case '2':
          current_joint_name = L_LINK_NAME_1;
          printf("left shoulder pitch\n");
          break;
        case '3':
          current_joint_name = L_LINK_NAME_2;
          printf("left shoulder roll\n");
          break;
        case '4':
          current_joint_name = L_LINK_NAME_3;
          printf("left elbow pitch\n");
          break;
        case '5':
          current_joint_name = L_LINK_NAME_4;
          printf("left elbow roll\n");
          break;
        case '6':
          current_joint_name = L_LINK_NAME_5;
          printf("left wrist pitch\n");
          break;
        case '7':
          current_joint_name = L_LINK_NAME_6;
          printf("left wrist roll\n");
          break;
        case '8':
          current_joint_name = L_LINK_NAME_7;
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
          current_joint_name = R_LINK_NAME_0;
          printf("right turret\n");
          break;
        case '2':
          current_joint_name = R_LINK_NAME_1;
          printf("right shoulder pitch\n");
          break;
        case '3':
          current_joint_name = R_LINK_NAME_2;
          printf("right shoulder roll\n");
          break;
        case '4':
          current_joint_name = R_LINK_NAME_3;
          printf("right elbow pitch\n");
          break;
        case '5':
          current_joint_name = R_LINK_NAME_4;
          printf("right elbow roll\n");
          break;
        case '6':
          current_joint_name = R_LINK_NAME_5;
          printf("right wrist pitch\n");
          break;
        case '7':
          current_joint_name = R_LINK_NAME_6;
          printf("right wrist roll\n");
          break;
        case '8':
          current_joint_name = R_LINK_NAME_7;
          printf("right gripper\n");
          break;
        case '9':
          printf("Resetting right commands to current position.\n");
        default:
          break;
      }
    }

    if (dirty == true) {
      dirty=false; // Sending the command only once for each key press.
      if(!right_arm) {
        publish(LEFT_ARM_COMMAND_TOPIC,lArmCmd);
        publish(LEFT_GRIPPER_COMMAND_TOPIC,lGripperCmd);
      } else {
        publish(RIGHT_ARM_COMMAND_TOPIC,rArmCmd);
        publish(RIGHT_GRIPPER_COMMAND_TOPIC,rGripperCmd);
      }
    }
  }
}


