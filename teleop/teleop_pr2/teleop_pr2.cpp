#include <cstdlib>
#include <unistd.h>
#include <math.h>
#include "ros/node.h"
#include "joy/Joy.h"
#include "robot_msgs/PoseDot.h"
#include "robot_msgs/JointCmd.h"
#include "std_msgs/Float64.h"

#define TORSO_TOPIC "/torso_lift_controller/set_command"
#define HEAD_TOPIC "/head_controller/set_command_array"

using namespace ros;

class TeleopBase : public Node
{
   public:
  robot_msgs::PoseDot cmd, cmd_passthrough;
  std_msgs::Float64 torso_vel;
  joy::Joy joy;
  double req_vx, req_vy, req_vw, req_torso, req_pan, req_tilt;
  double max_vx, max_vy, max_vw, max_vx_run, max_vy_run, max_vw_run;
  double max_pan, max_tilt, min_tilt, pan_step, tilt_step;
  int axis_vx, axis_vy, axis_vw, axis_pan, axis_tilt;
  int deadman_button, run_button, torso_dn_button, torso_up_button, head_button, passthrough_button;
  bool deadman_no_publish_, torso_publish, head_publish;
  ros::Time last_recieved_joy_message_time_;
  ros::Duration joy_msg_timeout_;
  // Set pan, tilt steps as params

  TeleopBase(bool deadman_no_publish = false) : Node("teleop_base"), max_vx(0.6), max_vy(0.6), max_vw(0.8), max_vx_run(0.6), max_vy_run(0.6), max_vw_run(0.8), max_pan(2.7), max_tilt(1.4), min_tilt(-0.4), pan_step(0.1), tilt_step(0.1), deadman_no_publish_(deadman_no_publish)
      {
        torso_vel.data = 0;
        cmd.vel.vx = cmd.vel.vy = cmd.ang_vel.vz = 0;
        req_pan = req_tilt = 0;
        if (!hasParam("max_vx") || !getParam("max_vx", max_vx))
          ROS_WARN("maximum linear velocity (max_vx) not set. Assuming 0.6");
        if (!hasParam("max_vy") || !getParam("max_vy", max_vy))
          ROS_WARN("maximum linear velocity (max_vy) not set. Assuming 0.6");
        if (!hasParam("max_vw") || !getParam("max_vw", max_vw))
          ROS_WARN("maximum angular velocity (max_vw) not set. Assuming 0.8");
        
        // Set max speed while running
        if (!hasParam("max_vx_run") || !getParam("max_vx_run", max_vx_run))
          ROS_WARN("maximum running linear velocity (max_vx_run) not set. Assuming 0.6");
        if (!hasParam("max_vy_run") || !getParam("max_vy_run", max_vy_run))
          ROS_WARN("maximum running linear velocity (max_vy_run) not set. Assuming 0.6");
        if (!hasParam("max_vw_run") || !getParam("max_vw_run", max_vw_run))
          ROS_WARN("maximum running angular velocity (max_vw_run) not set. Assuming 0.8");
        
        if (!hasParam("max_pan") || !getParam("max_pan", max_pan))
          ROS_WARN("maximum pan not set. Assuming 0.6");
        if (!hasParam("max_tilt") || !getParam("max_tilt", max_tilt))
          ROS_WARN("maximum tilt not set. Assuming 1.4");
        if (!hasParam("min_tilt") || !getParam("min_tilt", max_tilt))
          ROS_WARN("maximum tilt not set. Assuming -0.4");

        if (!hasParam("tilt_step") || !getParam("tilt_step", tilt_step))
          ROS_WARN("tilt step not set. Assuming 0.1");
        if (!hasParam("pan_step") || !getParam("pan_step", pan_step))
          ROS_WARN("pan step not set. Assuming 0.1");

        param<int>("axis_pan", axis_pan, 0);
        param<int>("axis_tilt", axis_tilt, 2);

        param<int>("axis_vx", axis_vx, 3);
        param<int>("axis_vw", axis_vw, 0);
        param<int>("axis_vy", axis_vy, 2);
        
        param<int>("deadman_button", deadman_button, 0);
        param<int>("run_button", run_button, 0);
        param<int>("torso_dn_button", torso_dn_button, 0);
        param<int>("torso_up_button", torso_up_button, 0);
        param<int>("head_button", head_button, 0);
        param<int>("passthrough_button", passthrough_button, 1);

	double joy_msg_timeout;
        param<double>("joy_msg_timeout", joy_msg_timeout, -1.0); //default to no timeout
	if (joy_msg_timeout <= 0)
	  {
	    joy_msg_timeout_ = ros::Duration().fromSec(9999999);//DURATION_MAX;
	    ROS_DEBUG("joy_msg_timeout <= 0 -> no timeout");
	  }
	else
	  {
	    joy_msg_timeout_.fromSec(joy_msg_timeout);
	    ROS_DEBUG("joy_msg_timeout: %.3f", joy_msg_timeout_.toSec());
	  }

        ROS_DEBUG("max_vx: %.3f m/s\n", max_vx);
        ROS_DEBUG("max_vy: %.3f m/s\n", max_vy);
        ROS_DEBUG("max_vw: %.3f deg/s\n", max_vw*180.0/M_PI);
        
        ROS_DEBUG("max_vx_run: %.3f m/s\n", max_vx_run);
        ROS_DEBUG("max_vy_run: %.3f m/s\n", max_vy_run);
        ROS_DEBUG("max_vw_run: %.3f deg/s\n", max_vw_run*180.0/M_PI);
        
        ROS_DEBUG("tilt step: %.3f rad\n", tilt_step);
        ROS_DEBUG("pan step: %.3f rad\n", pan_step);
        
        ROS_DEBUG("axis_vx: %d\n", axis_vx);
        ROS_DEBUG("axis_vy: %d\n", axis_vy);
        ROS_DEBUG("axis_vw: %d\n", axis_vw);
        ROS_DEBUG("axis_pan: %d\n", axis_pan);
        ROS_DEBUG("axis_tilt: %d\n", axis_tilt);
        
        ROS_DEBUG("deadman_button: %d\n", deadman_button);
        ROS_DEBUG("run_button: %d\n", run_button);
        ROS_DEBUG("torso_dn_button: %d\n", torso_dn_button);
        ROS_DEBUG("torso_up_button: %d\n", torso_up_button);
        ROS_DEBUG("head_button: %d\n", head_button);
        ROS_DEBUG("passthrough_button: %d\n", passthrough_button);
        ROS_DEBUG("joy_msg_timeout: %f\n", joy_msg_timeout);
        
        if (torso_dn_button != 0)
          advertise<std_msgs::Float64>(TORSO_TOPIC, 1);
        if (head_button != 0)
          advertise<robot_msgs::JointCmd>(HEAD_TOPIC, 1);

        advertise<robot_msgs::PoseDot>("cmd_vel", 1);
        subscribe("joy", joy, &TeleopBase::joy_cb, 1);
        subscribe("cmd_passthrough", cmd_passthrough, &TeleopBase::passthrough_cb, 1);
        ROS_DEBUG("done with ctor\n");
      }
  
  ~TeleopBase()
  {
    unsubscribe("joy");
    unsubscribe("cmd_passthrough");
    unadvertise("cmd_vel");

    if (torso_dn_button != 0)
      unadvertise(TORSO_TOPIC);
    if (head_button != 0)
      unadvertise(HEAD_TOPIC);

  }

      void joy_cb()
      {
	//Record this message reciept
	last_recieved_joy_message_time_ = ros::Time::now();

        bool cmd_head = (((unsigned int)head_button < joy.get_buttons_size()) && joy.buttons[head_button]);

        bool deadman = (((unsigned int)deadman_button < joy.get_buttons_size()) && joy.buttons[deadman_button]);
        
        // Base
        bool running = (((unsigned int)run_button < joy.get_buttons_size()) && joy.buttons[run_button]);
        double vx = running ? max_vx_run : max_vx;
        double vy = running ? max_vy_run : max_vy;
        double vw = running ? max_vw_run : max_vw;

         if((axis_vx >= 0) && (((unsigned int)axis_vx) < joy.get_axes_size()) && !cmd_head)
            req_vx = joy.axes[axis_vx] * vx;
         else
            req_vx = 0.0;
         if((axis_vy >= 0) && (((unsigned int)axis_vy) < joy.get_axes_size()) && !cmd_head)
            req_vy = joy.axes[axis_vy] * vy;
         else
            req_vy = 0.0;
         if((axis_vw >= 0) && (((unsigned int)axis_vw) < joy.get_axes_size()) && !cmd_head)
            req_vw = joy.axes[axis_vw] * vw;
         else
            req_vw = 0.0;

         // Head
         // Update commanded position by how joysticks moving
         // Don't add commanded position if deadman off
         if((axis_pan >= 0) && (((unsigned int)axis_pan) < joy.get_axes_size()) && cmd_head && deadman)
         {
           req_pan += joy.axes[axis_pan] * pan_step;
           req_pan = std::max(std::min(req_pan, max_pan), -max_pan);
         }

         if ((axis_tilt >= 0) && (((unsigned int)axis_tilt) < joy.get_axes_size()) && cmd_head && deadman)
         {
           req_tilt += joy.axes[axis_tilt] * tilt_step;
           req_tilt = std::max(std::min(req_tilt, max_tilt), min_tilt);
         }


         // Torso
         bool down = (((unsigned int)torso_dn_button < joy.get_buttons_size()) && joy.buttons[torso_dn_button]);
         bool up = (((unsigned int)torso_up_button < joy.get_buttons_size()) && joy.buttons[torso_up_button]);

         // Bring torso up/down with max effort
         if (down && !up)
           req_torso = -0.01;
         else if (up && !down)
           req_torso = 0.01;
         else
           req_torso = 0;

         
      }
      void passthrough_cb() { }
      void send_cmd_vel()
      {
         joy.lock();
         if(((deadman_button < 0) ||
            ((((unsigned int)deadman_button) < joy.get_buttons_size()) &&
             joy.buttons[deadman_button]))
	    &&
	    last_recieved_joy_message_time_ + joy_msg_timeout_ > ros::Time::now())
         {
            if (passthrough_button >= 0 &&
                passthrough_button < (int)joy.get_buttons_size() &&
                joy.buttons[passthrough_button])
            {
               // pass through commands that we have received (e.g. from wavefront)
               cmd_passthrough.lock();
               cmd = cmd_passthrough;
               cmd_passthrough.unlock();
            }
            else
            {
               // use commands from the local sticks
               cmd.vel.vx = req_vx;
               cmd.vel.vy = req_vy;
               cmd.ang_vel.vz = req_vw;
            }
            publish("cmd_vel", cmd);
            
            // Torso
            torso_vel.data = req_torso;
	    if (torso_dn_button != 0)
	      publish(TORSO_TOPIC, torso_vel);

            // Head
            if (head_button != 0)
            {
              robot_msgs::JointCmd joint_cmds ;
              joint_cmds.positions.push_back(req_pan);
              joint_cmds.positions.push_back(req_tilt);
              joint_cmds.velocity.push_back(0.0);
              joint_cmds.velocity.push_back(0.0);
              joint_cmds.acc.push_back(0.0);
              joint_cmds.acc.push_back(0.0);
              joint_cmds.names.push_back("head_pan_joint");
              joint_cmds.names.push_back("head_tilt_joint");
              publish(HEAD_TOPIC, joint_cmds);
            }

            if (req_torso != 0)
              fprintf(stderr,"teleop_base:: %f, %f, %f. Head:: %f, %f. Torso effort: %f.\n",cmd.vel.vx,cmd.vel.vy,cmd.ang_vel.vz, req_pan, req_tilt, torso_vel.data);
            else
              fprintf(stderr,"teleop_base:: %f, %f, %f. Head:: %f, %f\n",cmd.vel.vx,cmd.vel.vy,cmd.ang_vel.vz, req_pan, req_tilt);
         }
         else
         {
           cmd.vel.vx = cmd.vel.vy = cmd.ang_vel.vz = 0;
           torso_vel.data = 0;
           if (!deadman_no_publish_)
           {
             publish("cmd_vel", cmd);//Only publish if deadman_no_publish is enabled
	     if (torso_dn_button != 0)
	       publish(TORSO_TOPIC, torso_vel);

             // Publish head
             if (head_button != 0)
             {
               robot_msgs::JointCmd joint_cmds ;
               joint_cmds.positions.push_back(req_pan);
               joint_cmds.positions.push_back(req_tilt);
               joint_cmds.velocity.push_back(0.0);
               joint_cmds.velocity.push_back(0.0);
               joint_cmds.acc.push_back(0.0);
               joint_cmds.acc.push_back(0.0);
               joint_cmds.names.push_back("head_pan_joint");
               joint_cmds.names.push_back("head_tilt_joint");
               publish(HEAD_TOPIC, joint_cmds);
             }
             
           }
         }
         joy.unlock();
      }
};

int main(int argc, char **argv)
{
   ros::init(argc, argv);
   const char* opt_no_publish    = "--deadman_no_publish";

   bool no_publish = false;
   for(int i=1;i<argc;i++)
   {
     if(!strncmp(argv[i], opt_no_publish, strlen(opt_no_publish)))
       no_publish = true;
   }
   TeleopBase teleop_base(no_publish);
   while (teleop_base.ok())
   {
      usleep(50000);
      teleop_base.send_cmd_vel();
   }
   
   exit(0);
   return 0;
}

