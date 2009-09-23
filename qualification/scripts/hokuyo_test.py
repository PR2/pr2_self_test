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
import matplotlib
import matplotlib.pyplot as plot
import sys
from StringIO import StringIO
import threading
import time

from sensor_msgs.msg import LaserScan
from qualification.srv import TestResult, TestResultRequest
from qualification.msg import Plot

from optparse import OptionParser

def bool_to_msg(bool_val):
    if bool_val:
        return 'OK'
    return 'FAIL'

class TestParams:
    def __init__(self):

        self.transpose = False

        self.ok_noise     = 0.06
        self.histlen      = 10
        self.plotinterval = 5

        self.skip_count   = 20 # Skip first 20 scans


## This all requires the HK to be on its LEFT side
class LongRangeParams(TestParams):
    def __init__(self):
        TestParams.__init__(self)

        self.ok_err       = .1
        self.ok_maxerr    = .2

        self.tgt_a        = 0
        self.ok_a         = numpy.tan(5.0 * numpy.pi / 180) 
        self.tgt_b        = 5.6
        self.ok_b         = 1.0

        ##\todo Set angles correctly
        self.min_ang      = -15 * numpy.pi / 180
        self.max_ang      = -3 * numpy.pi / 180
    
class MidRangeParams(TestParams):
    def __init__(self):
        TestParams.__init__(self)

        # Transpose it since we're on the side
        self.transpose = True

        self.ok_err       = .01
        self.ok_maxerr    = .02
        self.tgt_a        = 0
        self.ok_a         = numpy.tan(3.0 * numpy.pi / 180) 
        self.tgt_b        = 2.0
        self.ok_b         = .05
        self.min_ang      = -110 * numpy.pi / 180
        self.max_ang      = -70 * numpy.pi / 180

class ShortRangeParams(TestParams):
    def __init__(self):
        TestParams.__init__(self)

        self.transpose = True

        self.ok_err       = .01
        self.ok_maxerr    = .02
        self.tgt_a        = 0
        self.ok_a         = numpy.tan(5.0 * numpy.pi / 180) 
        self.tgt_b        = -0.04
        self.ok_b         = .15
        self.min_ang      = 70 * numpy.pi / 180
        self.max_ang      = 110 * numpy.pi / 180


class HokuyoTest:
    def __init__(self, topic, param):
        self._mutex = threading.Lock()

        rospy.init_node('hokuyo_test')
        rospy.Subscriber(topic, LaserScan, self.callback)
        self.result_service = rospy.ServiceProxy('/test_result', TestResult)
        self.skipped = 0
        self.param = param
        self.xhist = []
        self.yhist = []
        self.rhist = []
        self.ihist = []

        self.plot_index = 0

        self.rec_count = 0

        self.data_sent = False

        
    def plotdata(self, x, y, a, b):
        self.plot_index += 1
        fig = plot.figure(self.plot_index)
        plot.ylabel('Y data')
        plot.xlabel('X data')

        plot.plot(x, y, 'b--', label='Data')
        #plot.plot(x, a + b * x, label='Fit')
        plot.axhline(y = self.param.tgt_b, color = 'g', label='Expected')
        fig.text(.38, .95, 'Hokuyo Range Data')
        plot.legend(shadow=True)

        

        stream = StringIO()
        plot.savefig(stream, format="png")
        image = stream.getvalue()
        p = Plot()
        p.title = 'range_data'
        p.image = image
        p.image_format = 'png'

        return p

    ##\brief Record errors in analysis
    def test_failed_service_call(self, except_str = ''):
        rospy.logerr(except_str)
        r = TestResultRequest()
        r.html_result = except_str
        r.text_summary = 'Caught exception, automated test failure.'
        r.plots = []
        r.result = TestResultRequest.RESULT_FAIL
        self.send_results(r)

    ##\brief Send test results to qualification system
    def send_results(self, test_result):
        if self.data_sent:
            return 

        rospy.wait_for_service('/test_result', 15)
        self.result_service.call(test_result)
        self.data_sent = True

    def plotintensity(self, x, intensity):
        self.plot_index += 1
        fig = plot.figure(self.plot_index)
        plot.ylabel('Intensity')
        plot.xlabel('X value')
        plot.plot(x, intensity, 'b--', label='Data')
        #plot.axhline(y = self.param.tgt_b, color = 'g', label='Expected')
        fig.text(.38, .95, 'Hokuyo Intensity Data')

        
                
        stream = StringIO()
        plot.savefig(stream, format="png")
        image = stream.getvalue()
        p = Plot()
        p.title = 'intensity_data'
        p.image = image
        p.image_format = 'png'

        return p

    

    def callback(self, data):
        if self.data_sent:
            return

        self.rec_count += 1
        if self.rec_count < self.param.skip_count:
            return

        self._mutex.acquire()
        # Create the angle and range arrays
        angles = numpy.arange(data.angle_min, data.angle_max, data.angle_increment);
        ranges = numpy.array(data.ranges)
        intensities = numpy.array(data.intensities)
    
        # Convert to cartesian
        xvals = []
        yvals = []
        rvals = []
        ivals = []
        for i in range(0,len(angles)):
            if angles[i] < self.param.min_ang or angles[i] > self.param.max_ang:
                continue

            if self.param.transpose:
                xvals.append(numpy.cos(angles[i])*ranges[i])
                yvals.append(-numpy.sin(angles[i])*ranges[i])
            else:
                xvals.append(-numpy.sin(angles[i])*ranges[i])
                yvals.append(numpy.cos(angles[i])*ranges[i])
            
            rvals.append(ranges[i])
            ivals.append(intensities[i])

        # Keep a history of the last 
        self.xhist.append(xvals)
        self.yhist.append(yvals)
        self.rhist.append(rvals)
        self.ihist.append(ivals)
        self._mutex.release()


    def analyze_data(self):
        self._mutex.acquire()

        mean_f = lambda l:sum(l)/len(l)
        xmean = map(mean_f,zip(*self.xhist))
        ymean = map(mean_f,zip(*self.yhist))
        imean = map(mean_f,zip(*self.ihist))
        
        square_f = lambda l: map(lambda x: pow(x,2),l)

        rhistt = zip(*self.rhist)
        r2histt = map(square_f, rhistt)
        r2mean = map(mean_f,r2histt)
        rmean = map(mean_f,rhistt)
        #noise = mean_f(map(lambda x2,x:numpy.sqrt(x2-pow(x,2)),r2mean,rmean))

        (a,b) = numpy.polyfit(xmean, ymean, 1)
        
        # Compute the error
        err = numpy.sqrt(sum(map(lambda x,y: (y-(a*x+b))**2, xmean, ymean))/ len(xmean))
        maxerr = numpy.sqrt(max(map(lambda x,y: (y-(a*x+b))**2, xmean, ymean)))

        # Go/no go
        err_ok = err < self.param.ok_err
        maxerr_ok = maxerr < self.param.ok_maxerr
        angle_ok = abs(self.param.tgt_a - a) < self.param.ok_a
        range_ok = abs(self.param.tgt_b - b) < self.param.ok_b

        table_error = '<table border="1" cellpadding="2" cellspacing="0">\n'
        table_error += '<tr><td><b>Param</b></td><td><b>Message</b></td><td><b>Value</b></td><td><b>Tolerance</b></td></tr>\n'
        table_error += '<tr><td>Std. Dev. of Wall</td><td>%s</td><td>%.3f</td><td>%.3f</td></tr>\n' % (bool_to_msg(err_ok), err, self.param.ok_err)
        table_error += '<tr><td>Max Error of Wall</td><td>%s</td><td>%.3f</td><td>%.3f</td></tr>\n' % (bool_to_msg(maxerr_ok), maxerr, self.param.ok_maxerr)
        table_error += '</table>\n'

        
        table_range = '<table border="1" cellpadding="2" cellspacing="0">\n'
        table_range += '<tr><td><b>Param</b></td><td><b>Message</b></td><td><b>Value</b></td><td><b>Expected</b></td><td><b>Tolerance</b></td></tr>\n'

        table_range += '<tr><td>Angle of Wall</td><td>%s</td><td>%.2f</td><td>%.2f</td><td>%.2f</td></tr>\n' % (bool_to_msg(angle_ok), a, self.param.tgt_a, self.param.ok_a)
        table_range += '<tr><td>Range of Wall</td><td>%s</td><td>%.2f</td><td>%.2f</td><td>%.2f</td></tr>\n' % (bool_to_msg(range_ok), b, self.param.tgt_b, self.param.ok_b)
        table_range += '</table>\n'
        
        html = '<img src=\"IMG_PATH/range_data.png\", width=640, height=480 />'
        html += table_error + '<br><br>\n' + table_range + '<br>\n'
        html += '<img src=\"IMG_PATH/intensity_data.png\", width=640, height=480 />'

        
        r = TestResultRequest()
        
        # Find pass/fail
        all_ok = err_ok and maxerr_ok and angle_ok and range_ok

        if all_ok:
            r.result = TestResultRequest.RESULT_PASS
            r.text_summary = 'Hokuyo range test: PASS' 
        else:
            r.result = TestResultRequest.RESULT_FAIL
            r.text_summary = 'Hokuyo range test failed. Error: %s, Max error: %s, Angle: %s, Range: %s' % \
                (bool_to_msg(err_ok), bool_to_msg(maxerr_ok), bool_to_msg(angle_ok), bool_to_msg(range_ok))
        

        r.plots = [ self.plotdata(xmean, ymean, a, b), self.plotintensity(xmean, imean) ]

        r.html_result = html

        self.send_results(r)
        

        self._mutex.release()
        

if __name__ == '__main__':
    parser = OptionParser(usage="./%prog [options]. Either short-, mid- or long- range.", prog="hokuyo_test.py")
    parser.add_option("--long", action="store_true", dest="long", default=False, metavar="LONGRANGE",
                      help="Long range test")
    parser.add_option("--mid", action="store_true", dest="mid", default=False, metavar="MIDRANGE",
                      help="Mid range test")
    parser.add_option("--short", action="store_true", dest="short", default=False, metavar="SHORTRANGE",
                      help="Short range test")

    options, args = parser.parse_args()

    multi_msg = "Options --short, --mid and --long are mutually exclusive"

    if not options.long and not options.mid and not options.short:
        parser.error("Must select a range to test")
    if options.long and options.mid:
        parser.error(multi_msg)
    if options.long and options.short:
        parser.error(multi_msg)
    if options.short and options.mid:
        parser.error(multi_msg)


    if options.long:
        ht = HokuyoTest('scan', LongRangeParams())
    if options.mid:
        ht = HokuyoTest('scan', MidRangeParams())
    else:
        ht = HokuyoTest('scan', ShortRangeParams())

    try:
        while len(ht.xhist) < ht.param.histlen and not rospy.is_shutdown():
            time.sleep(1)
        ht.analyze_data()
    except KeyboardInterrupt:
        raise
    except:
        import traceback
        rospy.logerr('Exception in hokuyo range test.\n%s' % traceback.format_exc())
        ht.test_failed_service_call(traceback.format_exc())
    
# Todo - write params (val, expected, tol)
# Setup HK URDF to show laser view in rviz
