#!/usr/bin/python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

PKG = 'qualification' 

import roslib; roslib.load_manifest(PKG)
import rospy
import serial
import random              
import threading
import traceback

from pr2_self_test_msgs.srv import TestResult, TestResultRequest


def index_to_msg(lst, i):
    try:
        if lst[i]:
            return "OK"
        return "FAIL"
    except:
        return "N/A"

class TestSerialPort:
    def __init__(self, port, rate, timeout):
        self.port = port
        self.serial = serial.Serial(port=port, baudrate=rate)
        self.timeout = timeout
        self.ok = [False] * 2
        self.thread = [None] * 2
        self.serial.timeout = 0
        while self.serial.read(1024) != '': # Clear the input buffer
            pass
        self.serial.timeout = timeout

        self.write_ok = False
        self.join_ok = False

    READ = 0
    WRITE = 1
    DIR = { 0: 'read', 1: 'write' }

    def start(self, direction, seed, length):
        rospy.logout( "start %s %s"%(self.port, TestSerialPort.DIR[direction]))
        self.ok[direction] = True
        self.thread[direction] = threading.Thread(target=self._run_test,args=[direction, seed, length])
        self.thread[direction].start()

    def join_is_error(self, direction):
        rospy.logout( "join %s %s"%(self.port, TestSerialPort.DIR[direction]))
        self.thread[direction].join(self.timeout + 5)
        if self.thread[direction].isAlive():
            rospy.logout( "Failed to join on %s test for %s"%(TestSerialPort.DIR[direction], self.port))
            self.ok[direction] = False
        return self.ok[direction]

    def _run_test(self, direction, seed, length):
        rospy.logout( "run %s %s"%(self.port, TestSerialPort.DIR[direction]))
        anyletter = [chr(i) for i in range(0,256)] 
        data = ''.join([random.Random(seed).choice(anyletter) for i in range(0,length)]) 
        if direction == TestSerialPort.READ:
            indata = self.serial.read(length)  
            if len(indata) != length:
                rospy.logout( "Read %i bytes instead of %i in %s"%(len(indata), length, self.port))
                return
            if data != indata:
                rospy.logout( "Input data did not match expectations for %s"%(self.port))
                return
        elif direction == TestSerialPort.WRITE:
            outlen = self.serial.write(data)
            # For now we can't check that the write occurred because write
            # always returns None. The documentation says it should return
            # the number of bytes written. I have filed a bug report with
            # upstream. Doesn't matter much for us, as if the bytes don't
            # go out, they won't come in.
            #if outlen != length:
            #    rospy.logout( "Wrote %i bytes instead of %i in %s"%(outlen, length, self.port))
            #    return
        else:
            raise ValueError
        # We only get here on success
        self.ok[direction] = True


class QualUSBSerial:
    def __init__(self):
        rospy.init_node('usb_serial_test')
        self.data_sent = False
        
        self.result_service = rospy.ServiceProxy('test_result', TestResult)

        self.reads = []
        self.writes = []

        self.ports = []
        self.names = []
        
    def failure_call(self, exception_str = ''):
        rospy.logerr(exception_str)
        r = TestResultRequest()
        r.html_result = exception_str
        r.text_summary = 'Caught exception, automated test failure.'
        r.plots = []
        r.result = TestResultRequest.RESULT_FAIL
        self.send_results(r)
        rospy.logout( 'Test failed, waiting for shutdown')
        rospy.spin()

    def send_results(self, test_result):
        if self.data_sent:
            return
        
        rospy.wait_for_service('test_result', 15)
        self.result_service.call(test_result)
        self.data_sent = True

    def run_qual_test(self):
                          
        bytes = 16384
        rate = 115200
        
        ports = []
        seeds = []
        
        timeout = bytes * 10 / rate * 1.1 + 3

        ok = True
        try:
            for i in range(0,4):
                self.names.append('/dev/ttyUSB%i'%i)
                try:
                    ports.append(TestSerialPort(self.names[i], rate, timeout))
                except serial.SerialException:
                    rospy.logout( 'couldnt open port: %d' % i)
                    self.failure_call("Error opening serial port %s. Device may not be connected." % self.names[i])

                seeds.append(random.random())
    
            for i in range(0,4):
                ports[i].start(TestSerialPort.READ, seeds[i^1], bytes)
                
            for i in range(0,4):
                ports[i].start(TestSerialPort.WRITE, seeds[i], bytes)
                
            for i in range(0,4):
                self.reads.append(ports[i].join_is_error(TestSerialPort.WRITE))
                self.writes.append(ports[i].join_is_error(TestSerialPort.READ))

        except serial.SerialException:
            self.failure_call(traceback.format_exc())
        except KeyboardInterrupt:
            self.failure_call(traceback.format_exc())

        r = TestResultRequest()
        r.text_summary = 'USB to Serial Test: %s' % self._passed_msg()
        r.html_result = self._write_results()
        r.result = TestResultRequest.RESULT_FAIL
        if self._passed():
            r.result = TestResultRequest.RESULT_PASS
        
        self.send_results(r)
        
    def _passed_msg(self):
        if len(self.reads) != len(self.writes) or len(self.reads) != 4:
            return "Incomplete"

        if self._passed():
            return "PASS"
        return "FAIL"

    def _passed(self):
        if len(self.reads) != len(self.writes) or len(self.reads) != 4:
            return False

        ok = True
        for read in self.reads:
            ok = ok and read
        for write in self.writes:
            ok = ok and write

        if ok:
            return "PASS"
        return "FAIL"

    def _write_results(self):
        html = '<p>USB to serial send/receive test: %s</p>\n' % self._passed()
        html += '<table border="1" cellpadding="2" cellspacing="0">\n'
        html += '<tr><td><b>Port</b></td><td><b>Read</b></td><td><b>Write</b></td></tr>\n'
        for i in range(0, 4):
            html += '<tr><td>%s</td><td>%s</td><td>%s</td></tr>\n' % (self.names[i], index_to_msg(self.reads, i), index_to_msg(self.writes, i))
        html += '</table>\n'

        return html
        
        

if __name__ == '__main__':
    qt = QualUSBSerial()
    try:
        qt.run_qual_test()
    except:
        qt.failure_call(traceback.format_exc())


