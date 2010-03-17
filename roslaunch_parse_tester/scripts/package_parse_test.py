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
##\brief Checks all roslaunch files in a package can parse and load params

PKG = 'roslaunch_parse_tester'
import roslib; roslib.load_manifest(PKG)

from roslaunch_parse_tester.package_parse import ROSLaunchPackageParser

from optparse import OptionParser
import sys, os
import subprocess

import signal

def shutdown(sig, stackframe):
    print 'Caught SIGINT, exiting'
    sys.exit(-1)

if __name__=='__main__':
    parser = OptionParser(usage="./%prog PKG [options]", 
                          prog="package_parse_test.py")
    parser.add_option("-v", "--verbose", action="store_true",
                      dest="verbose", default=False,
                      help="Verbose output from roslaunch all launch files.")
    parser.add_option("-q", "--quiet", action="store_true",
                      dest="quiet", default=False,
                      help="Quiet output from for all results for all launch files.")
    parser.add_option("-a", "--check_all", action="store_true",
                      dest="check_all", default=False,
                      help="Check all file, proceed anyway on failures")
    parser.add_option("--env", action="append", default=[],
                      dest="environment", metavar="ENV,VAL",
                      help="Environment variables,values for launch files")
    parser.add_option("-b", "--blacklist", action="append", default=[],
                      dest="blacklist", metavar="BLACKLIST",
                      help="Known bad files. Give path from package")
    parser.add_option("--black_dir", metavar="BLACK_DIR",
                      dest="black_dirs", default=[], action="append",
                      help="Known bad directories, all files ignored.")
    parser.add_option("-m", "--machine", action="store_true",
                      dest="machine", default=False, 
                      help="Check that machines can be assigned")
    parser.add_option("-n", "--node_check", default=False,
                      action="store_true", dest="node_check",
                      help="Check that node exists")
    parser.add_option("-c", "--config_errors", action="store_true",
                      dest="check_config_errors", 
                      help="Check configuration errors")
    parser.add_option("-d", "--depends", action="store_true",
                      dest="check_deps",
                      help="Check for missing dependencies")

    options, args = parser.parse_args()

    if len(args) < 1:
        parser.error("Must specify one package to check")
        sys.exit(2)
    
    env = {}
    for val in options.environment:
        name, value = val.split(',')
        env[name] = value

    package = args[0]

    package_parser = ROSLaunchPackageParser(package, verbose = options.verbose, quiet = options.quiet,
                                            environment = env, blacklist = options.blacklist, check_all = options.check_all,
                                            black_dirs = options.black_dirs, assign_machines = options.machine, 
                                            node_check = options.node_check, config_err_check = options.check_config_errors,
                                            depend_check = options.check_deps)
    
    ok = package_parser.check_package()
    
    if ok:    
        print 'All launch files in package \'%s\' passed.' % package
        sys.exit(0)
    else:
        print 'Launch files failed to parse/load for package \'%s\'.' % package
        sys.exit(1)


    
