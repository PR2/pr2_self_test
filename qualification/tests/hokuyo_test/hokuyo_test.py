#! /usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008-2009, Willow Garage, Inc.
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

import roslib
roslib.load_manifest('qualification')
import rospy

import numpy
import sys
#import wxmpl
#import matplotlib

from laser_scan.msg import LaserScan

class TestParams:
    def __init__(self):
        self.ok_err       = .006
        self.ok_maxerr    = .025
        self.tgt_a        = 0
        self.ok_a         = numpy.tan(5 * numpy.pi / 180) # was 0.1
        self.tgt_b        = .275
        self.ok_b         = .01 
        self.min_ang      = -55 * numpy.pi / 180
        self.max_ang      = 55 * numpy.pi / 180
        self.ok_noise     = 0.006
        self.histlen      = 10
        self.plotinterval = 5

class HokuyoTest:
    def __init__(self, topic, param):
        rospy.Subscriber(topic, LaserScan, self.callback)
        self.skipped = 0
        self.param = param
        self.xhist = []
        self.yhist = []
        self.rhist = []

    def plotdata(self, x, y, xa, ya, a, b):
        print 'set terminal x11 1'
        print 'set xrange [-.5:.5]'
        print 'set yrange [0:.4]'
        print 'plot "-" using 1:2 with lines, "-" using 1:2 with lines, %f * x + %f'%(a,b)
        for i in range(0, len(x)):
            print x[i], y[i]
        print 'e'
        for i in range(0, len(xa)):
            print xa[i], ya[i]
        print 'e'

    def plotintensity(self, r, intensity):
        print 'set terminal x11 2'
        print 'set xrange [0:2]'
        print 'set yrange [0:10000]'
        print 'plot "-" using 1:2 with lines'
        for i in range(0, len(r)):
            print r[i], intensity[i]
        print 'e'

    def callback(self, data):
        # Create the angle and range arrays
        angles = numpy.arange(data.angle_min, data.angle_max, data.angle_increment);
        ranges = numpy.array(data.ranges)
        #intensities = numpy.array(data.intensities)
    
        # Convert to cartesian
        xvals = []
        yvals = []
        rvals = []
        #ivals = []
        for i in range(0,len(angles)):
            if angles[i] < self.param.min_ang or angles[i] > self.param.max_ang:
                continue
            xvals.append(-numpy.sin(angles[i])*ranges[i])
            yvals.append(numpy.cos(angles[i])*ranges[i])
            rvals.append(ranges[i])
            #ivals.append(intensities[i])

        # Keep a history of the last 
        self.xhist.append(xvals)
        self.yhist.append(yvals)
        self.rhist.append(rvals)
        if len(self.xhist) > self.param.histlen:
            self.xhist.pop(0)
            self.yhist.pop(0)
            self.rhist.pop(0)
        
        # Compute average over history
        if len(self.xhist) < 1:
            return
        mean_f = lambda l:sum(l)/len(l)
        xmean = map(mean_f,zip(*self.xhist))
        ymean = map(mean_f,zip(*self.yhist))
        square_f = lambda l: map(lambda x: pow(x,2),l)

        rhistt = zip(*self.rhist)
        r2histt = map(square_f, rhistt)
        r2mean = map(mean_f,r2histt)
        rmean = map(mean_f,rhistt)
        noise = mean_f(map(lambda x2,x:numpy.sqrt(x2-pow(x,2)),r2mean,rmean))

        # Find the linear best fit
        (a,b) = numpy.polyfit(xvals, yvals, 1)
        
        # Compute the error
        err = numpy.sqrt(sum(map(lambda x,y: (y-(a*x+b))**2, xvals, yvals))/ len(xvals))
        maxerr = numpy.sqrt(max(map(lambda x,y: (y-(a*x+b))**2, xvals, yvals)))

        # Go/no go
        if ( err > self.param.ok_err ):
          print >> sys.stderr, 'Measured std of wall is too high'
          
        if ( maxerr > self.param.ok_maxerr ):
          print >> sys.stderr, 'Worst outlier is too far off fit'
          
        if ( noise > self.param.ok_noise ):
          print >> sys.stderr, 'Noise is too high'
          
        if abs(self.param.tgt_a - a) > self.param.ok_a:
          print >> sys.stderr, 'Wrong angle'
                
        if abs(self.param.tgt_b - b) > self.param.ok_b:
          print >> sys.stderr, 'Wrong distance'
        
        print >> sys.stderr, "A: %6.3f theta: %8.3f deg B: %8.3f m stdev: %f m maxerr %f m noise %f" % \
                (a, 180 * numpy.arctan(a) / numpy.pi, b, err, maxerr, noise)

        # Plot occasionally
        self.skipped = self.skipped + 1
        if self.skipped >= self.param.plotinterval:
            #self.plotintensity(rvals, ivals)
            self.plotdata(xvals, yvals, xmean, ymean, a, b)
            self.skipped = 0

if __name__ == '__main__':
    rospy.init_node('hokuyo_test')
    
    ht = HokuyoTest('scan', TestParams())
    while not rospy.is_shutdown():
        rospy.sleep(1)
