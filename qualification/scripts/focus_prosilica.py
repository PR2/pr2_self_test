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

##\author Blaise Gassend
##\brief Displays prosilica focus on gnuplot window


import roslib
roslib.load_manifest('camera_focus')
import sys
import rospy
import cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from opencv_latest.cv_bridge import CvBridge, CvBridgeError
import subprocess
from prosilica_camera.srv import PolledImage, PolledImageRequest, PolledImageResponse
from time import sleep

class ImageConverter:
  def __init__(self):
    self.image_sub = rospy.Subscriber("prosilica/image", Image, self.process_image)
    self.bridge = CvBridge()
    null = open('/dev/null', 'w')
    self.gnuplot = subprocess.Popen("gnuplot", 
            stdin=subprocess.PIPE) #, stdout=null) 
    cv.NamedWindow("Image window", 1)

  def process_image(self, image):
    try:
      cv_image = self.bridge.imgmsg_to_cv(image, "mono8")
    except CvBridgeError, e:
      import traceback
      traceback.print_exc()

    (width, height) = cv.GetSize(cv_image)
    
    blurradius = 30
    origimg = cv.CreateMat(height, width, cv.CV_32FC1)
    cv.Scale(cv_image, origimg, 1.0/255, 0.0)
    
    blurimg = cv.CreateMat(height, width, cv.CV_32FC1)
    cv.Smooth(origimg, blurimg, cv.CV_BLUR, blurradius, blurradius)
    blurimgsqr = cv.CreateMat(height, width, cv.CV_32FC1)
    cv.Pow(blurimg, blurimgsqr, 2)

    blursqr = cv.CreateMat(height, width, cv.CV_32FC1)
    cv.Pow(origimg, blursqr, 2)
    cv.Smooth(blursqr, blursqr, cv.CV_BLUR, blurradius, blurradius)

    cv.Sub(blursqr, blurimgsqr, blursqr)
    cv.Pow(blursqr, blursqr, -0.5)
    cv.Threshold(blursqr, blursqr, 8, 0, cv.CV_THRESH_TOZERO_INV)

    cv.Sub(origimg, blurimg, blurimg)
    cv.Abs(blurimg, blurimg)
    cv.Smooth(blurimg, blurimg, cv.CV_BLUR, blurradius, blurradius)
    #print >> sys.stderr, cv.Sum(blurimg)
    
    cv.ShowImage("Image window", origimg)
    cv.WaitKey(2)

    print >> self.gnuplot.stdin, 'set terminal x11 0'
    print >> self.gnuplot.stdin, 'unset logscale xy'
    print >> self.gnuplot.stdin, 'set yrange [0:0.4]'
    print >> self.gnuplot.stdin, 'plot "-" with lines'
    for i in range(width-1):
        ## FIXME Plot combination of all lines.
        print >> self.gnuplot.stdin, blurimg[0,i]
    print >> self.gnuplot.stdin, 'e'
 
def main(args):
  rospy.init_node('image_converter')
  ic = ImageConverter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  except:
    print "Shut down"
  cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
