#!/usr/bin/env python
import roslib; roslib.load_manifest('pr2_motor_diagnostic_tool')
import rospy
from pr2_motor_diagnostic_tool.srv import *
from pr2_mechanism_msgs.srv import LoadController, UnloadController, SwitchController, SwitchControllerRequest, ListControllers
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
import time
import sys
import argparse
from yaml import dump

load_controller = rospy.ServiceProxy('pr2_controller_manager/load_controller', LoadController)
unload_controller = rospy.ServiceProxy('pr2_controller_manager/unload_controller',UnloadController)
switch_controller = rospy.ServiceProxy('pr2_controller_manager/switch_controller',SwitchController)
list_controllers = rospy.ServiceProxy('pr2_controller_manager/list_controllers',ListControllers)

flag1 = False 
flag2 = False

#dictionary of actuators and their resistances
r_arm_actuators = ['r_wrist_r_motor','r_wrist_l_motor','r_forearm_roll_motor','r_upper_arm_roll_motor', 'r_elbow_flex_motor','r_shoulder_lift_motor','r_shoulder_pan_motor']
l_arm_actuators = ['l_wrist_r_motor','l_wrist_l_motor','l_forearm_roll_motor','l_upper_arm_roll_motor', 'l_elbow_flex_motor','l_shoulder_lift_motor','l_shoulder_pan_motor']
head_actuators = ['head_pan_motor','head_tilt_motor']

def wait_for_X():
    global flag1
    flag1 = False
    while not (flag1):
      rospy.sleep(0.01)

def wait_for_circle():
    global flag2
    flag2 = False
    while not (flag2):
      rospy.sleep(0.01) 

def start_diag_controller(actuator_name):
  rospy.set_param('diagnostic_controller/type', 'pr2_motor_diagnostic_controller/DiagnosticControllerPlugin')
  rospy.set_param('diag_actuator_name',str(actuator_name))
  resp = load_controller('diagnostic_controller')

  if resp.ok:
    rospy.loginfo("loaded controller dianostic_controller")
    switch_controller(['diagnostic_controller'],[],SwitchControllerRequest.STRICT)
    rospy.loginfo("Running diagnostic controller")
  else:
    raise RuntimeError("Couldn't load contorller")

def get_diag_data(actuator_name):
    #flag2 = False
    get_data = rospy.ServiceProxy('diagnostic_controller/get_diagnostic_data',DiagnosticData)
    rospy.loginfo("getting data for %s, wait 2 seconds",actuator_name)

    foo = DiagnosticDataRequest();
    rv = get_data(foo)
    stream = file(str(actuator_name) + "_results" + '.yaml', 'w')
    dump(rv,stream)

    switch_controller([],['diagnostic_controller'], SwitchControllerRequest.STRICT)
    rospy.loginfo("stopped diagnostic_controller")

    unload_controller("diagnostic_controller")
    rospy.loginfo("unload diagnostic_controller")


def callback(data):
  global flag1
  global flag2

  if (data.buttons[14] == 1):
    flag1 = True

  if (data.buttons[13] == 1): 
    flag2 = True

def main():
  rospy.init_node('get_data', anonymous=True)
  rospy.Subscriber("joy", Joy, callback)
  parser = argparse.ArgumentParser("script to get data for Pr2 arms for diagnostic analysis")
  parser.add_argument("parts", help="Specifiy left, right or both for the arms you want to get diagnostic data for and head for the head.")
  args = parser.parse_args()
  actuator_list = []
  if (args.parts == 'left'):
    actuator_list = l_arm_actuators
  elif (args.parts == 'right'): 
    actuator_list = r_arm_actuators
  elif (args.parts == 'both'):
    actuator_list = r_arm_actuators + l_arm_actuators
  elif (args.parts == 'head'):
    actuator_list = head
  else:
    print "Bad arguments, exiting"
    sys.exit()

  switch_controller([],['diagnostic_controller'],SwitchControllerRequest.STRICT)
  unload_controller('diagnostic_controller')

  for actuator_name in actuator_list:
    print "Press X to start or for next joint"
    wait_for_X() 
    start_diag_controller(actuator_name)
    print "start moving %s for about 5 seconds" %(actuator_name)
    rospy.sleep(5.0)
    print "press circle when your done"
    #rospy.loginfo("move %s for about 5 seconds and then press circle when done", joint_name)
    wait_for_circle()
    get_diag_data(actuator_name)
  
if __name__ == '__main__':
  try:
    #test_controller()
    main()
  except rospy.ROSInterruptException: pass

