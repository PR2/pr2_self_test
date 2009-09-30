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

#import roslib; roslib.load_manifest('qualification')
#import rospy
import serial
import random              
import threading

class TestSerialPort:
    def __init__(self, port, rate, timeout):
        self.port = port
        self.serial = serial.Serial(port=port, baudrate=rate)
        self.timeout = timeout
        self.error = [True] * 2
        self.thread = [None] * 2
        self.serial.timeout = 0
        while self.serial.read(1024) != '': # Clear the input buffer
            pass
        self.serial.timeout = timeout

    READ = 0
    WRITE = 1
    DIR = { 0: 'read', 1: 'write' }

    def start(self, direction, seed, length):
        print "start %s %s"%(self.port, TestSerialPort.DIR[direction])
        self.error[direction] = True
        self.thread[direction] = threading.Thread(target=self._run_test,args=[direction, seed, length])
        self.thread[direction].start()

    def join_is_error(self, direction):
        print "join %s %s"%(self.port, TestSerialPort.DIR[direction])
        self.thread[direction].join(self.timeout + 5)
        if self.thread[direction].isAlive():
            print "Failed to join on %s test for %s"%(TestSerialPort.DIR[direction], self.port)
            self.error[direction] = True
        return self.error[direction]

    def _run_test(self, direction, seed, length):
        print "run %s %s"%(self.port, TestSerialPort.DIR[direction])
        anyletter = [chr(i) for i in range(0,256)] 
        data = ''.join([random.Random(seed).choice(anyletter) for i in range(0,length)]) 
        if direction == TestSerialPort.READ:
            indata = self.serial.read(length)  
            if len(indata) != length:
                print "Read %i bytes instead of %i in %s"%(len(indata), length, self.port)
                return
            if data != indata:
                print "Input data did not match expectations for %s"%(self.port)
                return
        elif direction == TestSerialPort.WRITE:
            outlen = self.serial.write(data)
            # For now we can't check that the write occurred because write
            # always returns None. The documentation says it should return
            # the number of bytes written. I have filed a bug report with
            # upstream. Doesn't matter much for us, as if the bytes don't
            # go out, they won't come in.
            #if outlen != length:
            #    print "Wrote %i bytes instead of %i in %s"%(outlen, length, self.port)
            #    return
        else:
            raise ValueError
        # We only get here on success
        self.error[direction] = False

if __name__ == '__main__':
    fault = False
    bytes = 16384
    rate = 115200
    
    ports=[]
    seeds=[]
    names=[]
    timeout = bytes * 10 / rate * 1.1 + 3
    try:
        for i in range(0,4):
            names.append('/dev/ttyUSB%i'%i)
            try:
                ports.append(TestSerialPort(names[i], rate, timeout))
            except serial.SerialException:
                print "Error opening serial port %s"%names[i]
                fault = True
                raise
            seeds.append(random.random())
    
        for i in range(0,4):
            ports[i].start(TestSerialPort.READ, seeds[i^1], bytes)
        
        for i in range(0,4):
            ports[i].start(TestSerialPort.WRITE, seeds[i], bytes)
    
        for i in range(0,4):
            if ports[i].join_is_error(TestSerialPort.WRITE):
                print "Write test failed on port %s"%names[i]
                fault = True
            if ports[i].join_is_error(TestSerialPort.READ):
                print "Read test failed on port %s"%names[i]
                fault = True

    except serial.SerialException:
        pass
    except KeyboardInterrupt:
        print "Test interrupted."
        fault = True

    if fault:
        print "Test failed."
    else:
        print "Test passed."
