#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
#

##\author Derek King
##\brief Listens to /diagnostics, makes sure projector LED is operating properly
PKG = 'qualification'

import roslib
roslib.load_manifest(PKG)

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from std_srvs.srv import *

from qualification.srv import TestResult, TestResultRequest

import rospy
import threading
import thread
import traceback
from time import sleep
import wx
import socket

class Statistic:
    def __init__(self):
        self._sum = 0.0
        self._count = 0
    def get_avg(self):
        if self._count <= 0:
            #float('NaN')
            return None
        return (self._sum / self._count)
    def get_max(self):
        if self._count <= 0:
            return None
        return self._max
    def get_min(self):
        if self._count <= 0:
            return None
        return self._min
    def sample(self, value):
        if self._count == 0:
            self._max = value
            self._min = value
        else:
            self._max = max([self._max, value])
            self._min = min([self._min, value])
        self._sum += value
        self._count += 1
    def summary(self, name):
        if self._count == 0:
            return "%s : no data samples" % (name)
        else:
            return "%s : average=%f, min=%f, max=%f" %(name, self.get_avg(), self.get_min(), self.get_max())



class TemperatureMonitor:
    def __init__(self):
        self._mutex = threading.Lock()
        self._mutex.acquire()

        self._start_time = rospy.get_time()
        self._temp_stat = Statistic()
        self._connected = False

        self._update_time = 0
        self._ok = True
        self._summary = "No temperature data..."
        self._html = "No temperature data..."

        self._start_temp = None

        # test is based on parameters taken from test node's namespace
        self._dmm_address = rospy.get_param("~dmm_address")
        self._max_acceptable_temperature = rospy.get_param("~max_acceptable_temperature")
        self._max_temperature_rise = rospy.get_param("~max_temperature_rise")

        thread.start_new_thread(self._temperature_thread_func, ())
        self._mutex.release()


    def _read_line(self, sock):
        msg = ""
        while True:
            msg += sock.recv(1024)
            end = msg.find("\n")
            if end == len(msg)-1:
                return msg
            elif end != -1:
                print "warning, dropping %d byte tail of message. Tail='%s'"%(len(msg)-end, msg[end:])
                return msg[0:end-1]

    def _temperature_thread_func(self):
        try:
            self._temperature_thread_func_internal()
        except Exception, e:        
            traceback.print_exc()

    def _temperature_thread_func_internal(self):
        print "Temperature Thread Started..."
        self._mutex.acquire()        
        dmm_address = self._dmm_address
        max_acceptable_temperature = self._max_acceptable_temperature
        max_temperature_rise = self._max_temperature_rise
        self._mutex.release()

        try :
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            lxi_port = 5025
            sock.connect((dmm_address, 5025))
        except socket.error :
            self._mutex.acquire()        
            print "Socket error"
            self._ok = False
            self._html = "Error getting connection to DMM"
            self._mutex.release()
            return
        
        # We are now connected to scope
        self._mutex.acquire()        
        self._connected = True
        self._mutex.release()
        print "Connected to DMM"

        try : 
            sock.sendall("unit:temperature C\n")
            while self._ok:
                sock.sendall("measure:temperature? thermistor, 10000, 1, 100\n")
                response = self._read_line(sock)
                #print "Got DMM response = '%s'" % response
                self._mutex.acquire()
                self._update_time = rospy.get_time()
                try:
                    temp = float(response)
                    self._temp_stat.sample(temp)
                    if self._start_temp == None:
                        self._start_temp = temp                        
                    if temp > max_acceptable_temperature:
                        print "Overtemp"
                        if self._ok:
                            self._ok = False
                            self._html = "Measured temperture of %f celcius is higher than limit of %f celcius." % (temp, max_acceptable_temperature)
                    if (temp-self._start_temp) > max_temperature_rise:
                        if self._ok:
                            self._ok = False
                            self._html = "Temperature has risen too much. Started at %f C. Now at %f C." % (self._start_temp, temp)
                except ValueError:
                    if self._ok :
                        self._html = "Couldn't convert value returned by DMM : %s" % (response)
                        self._ok = False
                self._mutex.release()
        except RuntimeError:
            self._mutex.acquire()
            print "Runtime error"
            if self._ok :
                self._html = "Communication problem"
                self._ok = False
            self._mutex.release()
            
        
    def check_ok(self):
        self._mutex.acquire()
        if self._ok:
            current_time = rospy.get_time()
            # Fail if there is no temperature data within a couple seconds
            if (self._update_time == 0) and ((current_time - self._start_time) > 3):
                if self._connected:
                    self._html = "DMM has produced any temperature data"
                else:
                    self._html = "Could not connect to DMM"
                self._ok = False
                
            # Fail if data has stopped coming in
            if (self._update_time != 0) and ((current_time - self._update_time) > 3):
                self._html = "DMM stopped providing temperture data"
                self._ok = False

        ok = self._ok
        self._mutex.release()
        return ok


    def is_done(self):
        # This is done as soon as we want
        return True

    def collect_result(self, r):
        self._mutex.acquire()  

        if self._ok and (self._update_time == 0):
            self._html = "Test was stopped before any temperature data was collected."
            self._ok = False

        if self._ok:
            self._html= "PASS"

        self._html += "<br>\n %s" % self._temp_stat.summary("LED Temperature (Celcius)")
                    
        test_name = "Temperature monitor"
        summary = test_name + " " + ("passed" if (self._ok) else "failed") + ". "
        if not self._ok:
            r.result = TestResultRequest.RESULT_FAIL
        r.text_summary += summary
        r.html_result += "<h3>" + test_name + "</h3>\n" + self._html + "\n"
        self._mutex.release()




class FocusDialog:
    def __init__(self):
        self._mutex = threading.Lock()
        self._mutex.acquire()

        self._done = False
        self._ok = True

        thread.start_new_thread(self._focus_thread_func, ())

        self._mutex.release()

    def _focus_thread_func(self):
        ret = wx.MessageBox("Did you successfully manage to focus the projector?", "Projector Focus", wx.YES_NO)
        self._mutex.acquire()        
        self._done = True
        if ret == wx.NO:
            self._ok = False
        self._mutex.release()
        wx.MessageBox("Waiting for other tests to complete", "Waiting", wx.YES_NO)

    def check_ok(self):
        self._mutex.acquire()
        ok = self._ok
        self._mutex.release()
        return ok
    
    def is_done(self):
        self._mutex.acquire()
        done = self._done
        self._mutex.release()
        #print "focus done = ", done
        return done

    def collect_result(self, r):
        self._mutex.acquire()  
        msg = "Focusing passed\n"
        if not self._done:
            msg = "Focusing did not complete\n"
            self._ok = False
        elif not self._ok:
            msg = "Focusing failed\n"

        test_name = "Focusing step"
        summary = test_name + " " + ("passed" if (self._ok) else "failed") + ". "
        if not self._ok:
            r.result = TestResultRequest.RESULT_FAIL
        r.text_summary += summary
        r.html_result += "<h3>" + test_name + "</h3>\n" + msg + "\n"
        self._mutex.release()


class ProjectorMonitor:
    def __init__(self):
        self._mutex = threading.Lock()
        self._mutex.acquire()

        self._start_time = rospy.get_time()
        self._voltage_stat = Statistic()

        self._update_time = 0
        self._ok = True
        self._summary = "No diagnostic data..."
        self._html = "No diagnostic data..."


        # test is based on parameters taken from test node's namespace
        self._actuator = rospy.get_param("~actuator")
        self._duration = rospy.get_param("~duration")
        self._short_circuit_voltage = rospy.get_param("~short_circuit_voltage")
        self._projector_voltage = rospy.get_param("~projector_voltage")
        self._diode_clamp_voltage = rospy.get_param("~diode_clamp_voltage")
        self._measured_current = rospy.get_param("~measured_current")
        self._allowed_current_error = rospy.get_param("~allowed_current_error")

        self._diag_sub = rospy.Subscriber('/diagnostics', DiagnosticArray, self._diag_callback)

        self._mutex.release()

            

    def _diag_callback(self, msg):
        self._mutex.acquire()
        self._diag_callback_with_mutex(msg)
        self._mutex.release()

    def _diag_callback_with_mutex(self, msg):        
        # Don't don't need to update once status is no longer OK
        if not self._ok:
            return 

        name = 'EtherCAT Device (%s)' % self._actuator
        for stat in msg.status:
            if stat.name == name :
                self._update_time = rospy.get_time()

                #Do diagnostics report any error conditions?
                if stat.level != 0 :
                    self._html  = "Projector MCB Error. "
                    self._html += "Error level %d; Error Message '%s'" % (stat.level, stat.message)
                    self._ok = False
                    return

                # Find LED voltage and measured current in data array?
                voltage = None
                current = None
                for kv in stat.values:
                    if kv.key == "LED voltage":
                        voltage = float(kv.value)
                    elif kv.key == "Measured current":
                        current = float(kv.value)
                if voltage == None:
                    self._html = "LED voltage could not be found in diagnostics data."
                    self._ok = False
                    return 
                if current == None:
                    self._html = "Measured current could not be found in diagnostics data."
                    self._ok = False 
                    return 
                self._voltage_stat.sample(voltage)
                
                # Verify voltage and current values
                if voltage < self._short_circuit_voltage :
                    self._html  = "Measured LED voltage of %f is below minimum LED voltage. <br>\n" % voltage
                    self._html += "A voltage below %fV indicates a short circuit." % self._short_circuit_voltage
                    self._ok = False
                elif voltage < self._projector_voltage :
                    # LED voltage is good, check current
                    current_error = current - self._measured_current;
                    if (abs(current_error) > self._allowed_current_error):
                        self._html  = "Measured current is  %fA, expected value close to %fA. <br>\n" % (current, self._measured_current)
                        self._html += "Measured current error is %fA.  Max acceptable error is %fA." % (current_error, self._allowed_current_error)
                        self._ok = False
                elif voltage < self._diode_clamp_voltage:
                    self._html  = "A voltage measurement of %f volts is too high. <br>\n" % voltage
                    self._html += "A voltage between %fV and %fV indicates projector is not attached or power leads are reversed" % (self._projector_voltage, self._diode_clamp_voltage) 
                    self._ok = False
                else:
                    self._html  = "LED voltage measurement of %f volts is too high. <br>\n" % voltage
                    self._html += "A voltage value above of %fV indicates that neither clamping diode or LED is attached" % self._diode_clamp_voltage
                    self._ok = False
                    
                # Stop looping through data once correct actuator is found
                break
        return
    

    def check_ok(self):
        self._mutex.acquire()
        if self._ok:
            current_time = rospy.get_time()
            # Fail if there are no diagnostics within a couple seconds
            if (self._update_time == 0) and ((current_time - self._start_time) > 3):            
                self._html = "Didn't not receive any diagnostics after starting"
                self._ok = False
                
            # Fail if there is not diagnostics data has stopped coming in
            if (self._update_time != 0) and ((current_time - self._update_time) > 3):
                self._html = "Diagnostics are stale"
                self._ok = False

        ok = self._ok
        self._mutex.release()
        return ok

    def is_done(self):
        self._mutex.acquire()
        done = ((rospy.get_time() - self._start_time) > self._duration)
        self._mutex.release()
        #print "monitor done = ", done
        return done

    def collect_result(self, r):
        self._mutex.acquire()  

        if self._ok and (self._update_time == 0):
            self._html = "Test was stopped before any diagnostics data was collected."
            self._ok = False

        if self._ok:
            self._html= "PASS"

        self._html += "<br> %s" % self._voltage_stat.summary("LED voltage (volts)")            
        
        test_name = "Electrical test"
        summary = test_name + " " + ("passed" if (self._ok) else "failed") + ". "
        if not self._ok:
            r.result = TestResultRequest.RESULT_FAIL
        r.text_summary += summary
        r.html_result += "<h3>" + test_name + "</h3>\n" + self._html + "\n"
        self._mutex.release()
        

if __name__ == '__main__':
    try:
        rospy.init_node('projector_monitor')
        start_time = rospy.get_time()
        app = wx.PySimpleApp()
        result_service = rospy.ServiceProxy('test_result', TestResult)
        display_focus_dialog = rospy.get_param("~focus_dialog", False)        
        focus = None
        if (display_focus_dialog) :
            focus = FocusDialog()
        monitor = ProjectorMonitor()
        temperature = TemperatureMonitor()
        while not rospy.is_shutdown():
            if (not monitor.check_ok()):
                print "monitor failed"
                break
            if (not temperature.check_ok()):
                print "temperature failed"
                break
            if (focus != None) and (not focus.check_ok()):
                print "focus failed"
                break            
            if (monitor.is_done()) and (temperature.is_done()) and ((focus == None) or (focus.is_done())):
                break
            rospy.sleep(0.2)
 
        r = TestResultRequest()
        r.html_result = ''
        r.text_summary = ''
        r.plots = []
        r.result = TestResultRequest.RESULT_PASS

        monitor.collect_result(r)
        temperature.collect_result(r)
        if focus != None :
            focus.collect_result(r)
        print r
        rospy.wait_for_service('test_result', 2)
        result_service.call(r)
        
    except Exception, e:        
        traceback.print_exc()
        #sleep(100)
    except KeyError:
        traceback.print_exc()
        #sleep(100)
    rospy.loginfo('Quiting projector monitor, shutting down node.')
