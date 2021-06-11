#!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010, Willow Garage, Inc.
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

##\author Eric Berger, Kevin Watts
##\brief Determines counterbalance adjustments based on CB data

PKG = 'pr2_counterbalance_check'
from pr2_counterbalance_check import *


from joint_qualification_controllers.msg import CounterbalanceTestData

from optparse import OptionParser

import sys, os

from ros import rosrecord

import numpy

CB_MSG_TYPE = 'joint_qualification_controllers/CounterbalanceTestData'

def get_data(bag):
    for topic, msg, t in rosrecord.logplayer(bag):
        return CounterbalanceAnalysisData(msg)

def print_usage(code = 0):
    print("./counterbalance_training cb_bag1 cb_bag2 cb_bag3 ...")
    print("Determines counterbalance adjustments necessary to tune CB")
    print("Bags must have one message of type %s" % CB_MSG_TYPE)
    sys.exit(code)

if __name__ == '__main__':
    if len(sys.argv) < 2 or sys.argv[1] == '-h' or sys.argv[1] == '--help':
        print_usage(1)

    bags = sys.argv[1:]

    adjustments = {}
    efforts = {}
    print('Enter CB adjustments from "zero" in turns CW for each bag')
    for b in bags:
        if not os.path.exists(b):
            print("Bag %s does not exist. Check filename and retry" % b, file=sys.stderr)
            print_usage(1)

        #Ask for adjustments when this bag was taken
        try:
            adj_secondary = float(raw_input("Please enter secondary adjustment (turns CW) for bag %s: "%b))
            adj_cb_bar = float(raw_input("Please enter CB bar adjustment (turns CW) for bag %s: "%b))
            adjustments[b] = (adj_secondary, adj_cb_bar, 1)
        except:
            print("Invalid input for adjustment.  Floating point values expected.")
            sys.exit(1)

        data = get_data(b)
        efforts[b] = get_efforts(data, True) + get_efforts(data, False)
    
    B = numpy.array([efforts[b] for b in bags])
    A = numpy.array([adjustments[b] for b in bags])
    X = numpy.linalg.lstsq(A, B)
    model = X[0]
    model.dump('counterbalance_model.dat')
    print('\"counterbalance_model.dat\" contains CB adjustment values')
