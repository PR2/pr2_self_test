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
##\brief Checks given launch file for parsing and loading. 

PKG = 'roslaunch_parse_tester'

import roslib; roslib.load_manifest(PKG)

from launch_parse import ROSLaunchParser

import traceback

import subprocess

import os

class ROSLaunchPackageParser:
    def __init__(self, package, verbose = False, quiet = False, environment = {}, 
                 check_all = False, blacklist = [], black_dirs = [],
                 assign_machines = False, node_check = False, config_err_check = False,
                 depend_check = False):

        self.package = package
        self.verbose = verbose
        self.quiet = quiet
        self.environment = environment
        self.check_all = check_all

        pkg_dir = roslib.packages.get_pkg_dir(self.package)
        self.blacklist = [os.path.join(pkg_dir, blk) for blk in blacklist]
        self.black_dirs = [os.path.join(pkg_dir, dir) for dir in black_dirs]

        self.assign_machines = assign_machines
        self.node_check = node_check
        self.config_err_check = config_err_check
        self.depend_check = depend_check

        cmd = 'find `rospack find %s` -name \*.launch -print' % self.package
        p = subprocess.Popen(cmd, stdout = subprocess.PIPE,
                             stderr = subprocess.PIPE, shell=True)
        out, err = p.communicate()
        retcode = p.returncode
        
        if retcode != 0:
            raise 'Failed to run find files command. CMD: %s' % cmd
                                
        self.files = out.split('\n')

    def check_package(self):
        ok = True
        for file in self.files:
            if file is None or file == '':
                continue
        
            basename = os.path.basename(file) # Get path from package
        
            # Check blacklist and blacklist directories
            if file in self.blacklist:
                if not self.quiet:
                    print 'File %s is in blacklist, ignoring.' % basename
                continue

            blk_dir = False
            for dir in self.black_dirs:
                if file.startswith(dir):
                    blk_dir = True
                    break

            if blk_dir:
                if not self.quiet:
                    print 'File %s is in blacklist directory, ignoring.' % basename
                continue

            # Parse file, check if OK
            roslaunch_parser = ROSLaunchParser(file, verbose = self.verbose, 
                                               quiet = self.quiet, environment = self.environment)
        
            this_ok = True
            
            if not roslaunch_parser.parse_test():
                this_ok = False
                
            if this_ok and self.assign_machines and not roslaunch_parser.check_machines():
                this_ok = False
        
            if this_ok and self.node_check and not roslaunch_parser.check_nodes():
                this_ok = False

            if this_ok and self.config_err_check and not roslaunch_parser.check_config_errors():
                this_ok = False
            
            if this_ok and self.depend_check and not roslaunch_parser.check_missing_deps():
                this_ok = False
        
            ok = ok and this_ok
            
            if not self.quiet and this_ok:
                print 'Checked file %s: OK' % file
            elif not self.quiet:
                print 'Checked file %s: FAIL' % file
                
            if not this_ok and not self.check_all:
                if not self.quiet:
                    print 'Found bad file: %s. Aborting rest of checks' % basename
                break

        if ok:
            if not self.quiet:
                print 'Parse/load checks for package \'%s\' successful' % (self.package)
        if not ok:
            if not self.quiet:
                print 'Parse/load failure for launch files in package \'%s\'' % (self.package)

        return ok
        
