#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

##\author Kevin Watts
##\brief Programs and configures MCB's during component qualification

PKG = "qualification"
import roslib
roslib.load_manifest(PKG)
import rospy

import sys, os
import subprocess
from optparse import OptionParser
from time import sleep
import string

import traceback

from qualification.srv import ConfirmConf, ConfirmConfRequest, ConfirmConfResponse, ScriptDone, ScriptDoneRequest

import invent_client.invent_client
from invent_client.invent_client import Invent

prog_path = os.path.join(roslib.packages.get_pkg_dir(PKG), "fwprog")

class MCBProgramConfig:
    def __init__(self, expected):
        rospy.init_node('mcb_prog_conf')
        
        self.done_service = rospy.ServiceProxy('prestartup_done', ScriptDone)

        self.confirm_proxy = rospy.ServiceProxy('mcb_conf_results', ConfirmConf)
        self.has_finished = False

        username = rospy.get_param('/invent/username', None)
        password = rospy.get_param('/invent/password', None)

        if username == None or password == None:
            self.finished(False, 'Invalid username or password. Cannot check MCB\'s.')
            return

        self.invent = Invent(username, password)
        if not self.invent.login():
            self.finished(False, 'Unable to login to invent. Cannot check MCB\'s.')
            return 

        self.expected = expected

        self.serials = []


    def finished(self, pass_bool, msg = ''):
        if self.has_finished:
            return

        try:
            result = ScriptDoneRequest()
            if pass_bool:
                result.result = 0 # OK
            else:
                result.result = 1 # Fail

            result.failure_msg = msg
                
            rospy.wait_for_service('prestartup_done', 10)
            rospy.loginfo('Calling prestartup done service')
            self.done_service.call(result)
            self.has_finished = True
        except Exception, e:
            rospy.logerr(traceback.format_exc())


    def prompt_user(self, msg, details):
        rospy.loginfo('Prompting user: %s' % msg)

        conf = ConfirmConfRequest()
        conf.message = msg
        conf.details = details
        resp = self.confirm_proxy.call(conf)

        return resp.retry == ConfirmConfResponse.RETRY
        
    ## Counts, gets serials and verifies boards
    ##\brief param check bool : Whether to check if boards passed EE qualification
    def verify_boards(self, check):
        if self.has_finished:
            return False

        try:
            rospy.wait_for_service('mcb_conf_results', 10)
        except:
            rospy.logerr("MCB conf results service not available. Unable to configure MCB's")
            return False

        if not self.count_boards():
            return False

        if not self.check_link():
            return False

        if not self.get_serials():
            return False
        
        if check:
            return self.check_boards()
        return True

    ## Counts boards, returns true if passed
    def count_boards(self):
        count_cmd = os.path.join(prog_path, "eccount") + " -i eth0"

        while not rospy.is_shutdown():
            p = subprocess.Popen(count_cmd, stdout =subprocess.PIPE,
                                 stderr = subprocess.PIPE, shell=True)
            stdout, stderr = p.communicate()
            count = p.returncode
            details = 'Ran eccount. Expected %s MCB\'s. Return code %s.\n\n' % (self.expected, count)
            details += 'STDOUT:\n' + stdout
            if len(stderr) > 3:
                details += '\nSTDERR:\n' + stderr

                      
            if count == self.expected:
                rospy.logdebug("Found %s MCB's, programming" % count)
                return True

            elif count == 0:
                msg = "Found no MCB's! Check cables and power. Retry?"
            elif count == 200:
                msg = "Unable to initialize interface. Make sure you have root access and the link is connected. Check power to the device. Retry?"
            elif count == 203:
                msg = "Ethernet interface has link, but no packets are returning. This could mean the etherCAT cable is not plugged into an MCB. Check etherCAT cable. To debug, check that the first device is good. Retry?"
            elif count == 204:
                msg = "Has no link to device. Check device cable and power. Retry?"
            elif count == 205:
                msg = "Ethernet interface is not UP. The computer needs to be configured. Run \"sudo ifconfig eth0 up\" and click OK to retry."
            elif count > 199:
                msg = "Error counting MCB's. eccount gave error code: %s. Retry?" % count
            elif count < self.expected:
                msg = "Did not find all MCB's. Found %s, expected %s. Check cables to all MCB's after MCB %s. Click OK to retry." % (count, self.expected, count)
            elif count > self.expected:
                msg = "Found more MCB's than expected. Found %s, expected %s. Did you scan the right part? Click OK to retry." % (count, self.expected)
            else:
                msg = "MCB counts don't match. Found %s, expected %s. Retry?" % (count, self.expected)
                
            if not self.prompt_user(msg, details):
                self.finished(False, 'Failed to count boards. Operator canceled.\n\n%s\n%s' % (msg, details))
                return False

    def check_link(self):
        emltest_cmd = os.path.join(prog_path, "emltest") + " -q -j8 -ieth0 -T10000,2"

        p = subprocess.Popen(emltest_cmd, stdout = subprocess.PIPE, 
                             stderr = subprocess.PIPE, shell = True)
        o, e = p.communicate()
        retcode = p.returncode

        if retcode != 0:
            msg = "Dropped packets to device while checking link. Cables may be damaged."
            details = "Ran \"emltest\", found dropped packets when checking link. This is probably a cable problem from the test fixture or between some MCB\'s."
            
            self.finished(False, "%s\n\n%s" % (msg, details))
            return False

        return True


    def get_serials(self):
        for board in range(0, self.expected):
            check_cmd = os.path.join(prog_path, 'device') + ' -i eth0 -K -p %d' % (board + 1)
            try:
                p = subprocess.Popen(check_cmd, stdout = subprocess.PIPE,
                                     stderr = subprocess.PIPE, shell = True)
                stdout, stderr = p.communicate()
                retcode = p.returncode
                
                if retcode != 0:
                    self.finished(False, 'Error when checking board %d. "device" returned code %d\n\nOUT: %s\nERROR: %s' % (board, retcode, stdout, stderr))
                    return False
                
                for ln in stdout.split('\n'):
                    if ln.startswith('serial :'):
                        self.serials.append(int(ln[9:].strip()))
            except Exception, e:
                rospy.logerr(traceback.format_exc())
                self.finished(False, 'Error when checking board %d. Caught exception:\n%s' % (board, traceback.format_exc()))
                return False

        return True

                        
    def check_boards(self):
        # Get invent username/password
        for index, serial in enumerate(self.serials):
            rospy.logdebug('Checking serial %s' % serial)
            if not self.invent.get_test_status(serial):
                self.finished(False, 'Board %d has failed testing. Serial: %s' % (index, serial))
                return False

        return True

            
    ## Programs MCB's and calls result service when finished
    def program_boards(self):
        for board in range(0, self.expected):
            program_cmd = os.path.join(prog_path, "fwprog") + " -i eth0 -p %s %s/*.bit" % ((board + 1), prog_path)

            while not rospy.is_shutdown():
                p = subprocess.Popen(program_cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell=True)
                stdout, stderr = p.communicate()
                retcode = p.returncode

                details = 'Ran fwprog on MCB %s, returned %s.\n\n' % (board, retcode)
                details += 'CMD:\n' + program_cmd + '\n'
                details += 'STDOUT:\n' + stdout
                if len(stderr) > 5:
                    details += '\nSTDERR:\n' + stderr
                    
                if retcode == 0:
                    break
                
                msg = "Programming MCB firmware failed for MCB #%s with error %d! Would you like to retry?" %(board, retcode)
                retry = self.prompt_user(msg, details)
                if not retry:
                    self.finished(False, '%s\n%s' % (msg, details))
                    return False
                            
                
        self.finished(True, "Boards programmed. Num. boards: %d. Serials: %s" % (self.expected, str(self.serials)))
        return True
    
    def configure_boards(self, mcbs, assembly = False):
        path = roslib.packages.get_pkg_dir("ethercat_hardware")
        actuator_path = os.path.join(path, "actuators.conf")

        for mcb in mcbs:
            name, num = mcb.split(',')

            cmd = os.path.join(path, "motorconf") + " -i eth0 -p -n %s -d %s -a %s" % (name, num, actuator_path)

            while not rospy.is_shutdown():
                p = subprocess.Popen(cmd, stdout = subprocess.PIPE, stderr = subprocess.PIPE, shell = True)
                stdout, stderr = p.communicate()
                retcode = p.returncode
                
                details = 'Ran motorconf. Attempted to program MCB %s with actuator name %s. Return code: %s.\n\n' % (num, name, retcode)
                details += 'CMD:\n' + cmd + '\n'
                details += 'STDOUT:\n' + stdout
                if len(stderr) > 5:
                    details += '\nSTDERR:\n' + stderr
                    
                if retcode != 0:
                    msg = "Programming MCB configuration failed for %s with return code %s." % (name, retcode)
                    retry = self.prompt_user("%s Retry?" % msg, details)
                    if not retry:
                        self.finished(False, '%s\n%s' % (msg, details))
                        return
                else:
                    break

        self.update_conf(mcbs)

        if assembly and not self.check_assembly():
            self.finished(False, "Mismatch between given MCB's and listed items in inventory system. Component is not properly assembled in inventory. Found MCB serials: %s." % str(self.serials))


        self.finished(True, "Boards configured. Num. boards: %d. Serials: %s" % (self.expected, str(self.serials)))
        return

    def update_conf(self, mcbs):
        for index, serial in enumerate(self.serials):
            conf_name = mcbs[index].split(',')[0]
            self.invent.setKV(serial, 'Configuration', conf_name)

    def check_assembly(self):
        test_component = rospy.get_param('/qual_item/serial', None)
        if test_component is None:
            rospy.logerr('Unable to find serial number of test component')
            return False

        sub_components = self.invent.get_sub_items(test_component, recursive=True)
        if len(sub_components) == 0:
            rospy.logerr('Unable to find any sub components for item %s' % test_component)
            return False

        rospy.logdebug('Found sub components: %s' % str(sub_components))

        for serial in self.serials:
            if not str(serial) in sub_components:
                rospy.logerr('MCB serial %s was not listed under component %s in invent. Unit is incorrectly assembled in inventory system.' % (serial, test_component))
                return False

        return True

if __name__ == '__main__':
    parser = OptionParser(usage="./%prog [options]", prog='mcb_prog_conf.py')
    parser.add_option("-p", "--program", action="store_true", 
                      dest="program", default=False, 
                      metavar="PROGRAM", help="Program mcbs?")
    parser.add_option("-c", "--configure", action="store_true", 
                      dest="configure", default=False, 
                      metavar="CONFIGURE", help="Configure mcbs?")
    parser.add_option("-n", "--number", type="int", dest="expected",
                      default = 0,
                      metavar="NUMBER", help="Number of MCB's")
    parser.add_option("--motor", type="string", dest="mcbs", 
                      action="append", metavar="NAME,NUM", 
                      help="MCB's to configure by name and number")
    parser.add_option("-a", "--assembly", action="store_true",
                      default=False, dest="assembly",
                      help="Verify that boards are components of qual item")
        
    options, args = parser.parse_args()
    
    if options.program and options.configure:
        parser.error("Options \'program\' and \'configure\' are mutually exclusive")
        
    if not options.program and not options.configure:
        parser.error("Must choose to program or configure boards")

    if options.expected == 0:
        parser.error("Must have MCB's to program or configure")

    if options.configure and options.expected != len(options.mcbs):
        parser.error("Number of boards doesn't match with list given")

    prog_conf = MCBProgramConfig(options.expected)

    try:
        sleep(2) # Wait for boards to warm up

        if not prog_conf.verify_boards(options.program):
            sys.exit() # Exit code is taken care of by verify functions

        if options.program:
            prog_conf.program_boards()
        else: # Configure
            prog_conf.configure_boards(options.mcbs, options.assembly)
        
        # Wait to be killed
        rospy.spin()
    except KeyboardInterrupt:
        pass
    except Exception, e:
        rospy.logerr(traceback.format_exc())
        prog_conf.finished(False, 'Caught exception in prog_conf loop. %s' % traceback.format_exc())
        
            
