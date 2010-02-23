#!/usr/bin/env python
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
#

##\author Kevin Watts
##\brief Check given launch file for parsing and loading

PKG = 'roslaunch_parse_tester'
import roslib; roslib.load_manifest(PKG)

from roslaunch_parse_tester.launch_parse import ROSLaunchParser

from optparse import OptionParser
import sys

if __name__=='__main__':
    parser = OptionParser(usage="./%prog file [options]", 
                          prog="launch_parser.py")

    parser.add_option("-v", "--verbose", action="store_true",
                      dest="verbose", default=False,
                      help="Verbose output for roslaunch parsing")
    parser.add_option("-q", "--quiet", action="store_true",
                      dest="quiet", default=False,
                      help="Parser gives error explanations if not quiet")
    parser.add_option("--env", action="append", default=[],
                      dest="environment", metavar="ENV,VAL",
                      help="Environment variables,value for launch file")
    parser.add_option("-m", "--machine", action="store_true",
                      dest="machine", default=False, 
                      help="Check machine assignments for each node")
    parser.add_option("-n", "--node_check", default=False,
                      action="store_true", dest="node_check",
                      help="Check that node exists")
    parser.add_option("-c", "--config_errors", action="store_true",
                      dest="check_config_errors", 
                      help="Check configuration errors")
    
    options, args = parser.parse_args()

    if len(args) < 1:
        parser.error("Must specify one launch file to parse")
        sys.exit(2)
    
    env = {}
    for val in options.environment:
        name, value = val.split(',')
        env[name] = value

    roslaunch_parser = ROSLaunchParser(args[0], verbose=options.verbose, quiet=options.quiet, environment=env)
    
    if not roslaunch_parser.parse_test():
        print 'Failed to parse'
        sys.exit(1)
    if not roslaunch_parser.check_config_errors():
        print 'Configuration errors, parse failure'
        sys.exit(1)
    if options.machine and not roslaunch_parser.check_machines():
        print 'Machine check failed'
        sys.exit(1)
    if options.node_check and not roslaunch_parser.check_nodes():
        print 'Node check failed'
        sys.exit(1)

    print 'Launch file parsed successfully'
    sys.exit(0)
    
