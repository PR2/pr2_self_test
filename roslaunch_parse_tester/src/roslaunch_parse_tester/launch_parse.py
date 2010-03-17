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

import roslaunch.config
import roslaunch.xmlloader
import roslaunch.node_args 

import traceback
import stat
import os

# Shamelessly copied from roswtf/roslaunchwtf.py
def _find_node(pkg, node_type):
    try:
        dir = roslib.packages.get_pkg_dir(pkg)
    except roslib.packages.InvalidROSPkgException:
        # caught by another rule
        return []
    paths = []
    #UNIXONLY
    node_exe = None
    for p, dirs, files in os.walk(dir):
        if node_type in files:
            test_path = os.path.join(p, node_type)
            s = os.stat(test_path)
            if (s.st_mode & stat.S_IRWXU == stat.S_IRWXU):
                paths.append(test_path)
        if '.svn' in dirs:
            dirs.remove('.svn')
        elif '.git' in dirs:
            dirs.remove('.git')
    return paths

class ROSLaunchParser:
    def __init__(self, file, verbose = False, quiet = False, environment = {}):
        self.file = file
        self.quiet = quiet
        self.verbose = verbose

        self.config = None

        for env, val in environment.iteritems():
            os.environ[env] = val

    def parse_test(self):
        try:
            loader = roslaunch.xmlloader.XmlLoader()
            self.config = roslaunch.config.ROSLaunchConfig()

            loader.load(self.file, self.config, verbose=self.verbose)
            
            return True
        except roslaunch.xmlloader.XmlParseException, e:
            if not self.quiet:
                print 'Caught exception parsing ROSLaunch file'
                traceback.print_exc()
            return False
        except roslaunch.xmlloader.XmlLoadException, e:
            if not self.quiet:
                print 'Caught exception loading ROSLaunch file'
                traceback.print_exc()
            return False
        except roslaunch.core.RLException:
            if not self.quiet:
                print 'Caught RLException loading ROSLaunch file'
                traceback.print_exc()
            return False
        except:
            if not self.quiet:
                print 'Caught unknown exception parsing launch file'
                traceback.print_exc()
            return False

    def check_machines(self):
        if self.config is None:
            if not self.quiet:
                print 'Must parse launch file first.'
            return False
        try:
            self.config.assign_machines()
            return True
        except:
            if not self.quiet:
                print 'Caught exception attempting to assign machines'
                traceback.print_exc()
            return False

    ##\todo Make it so machines don't have to be assigned before checking nodes
    ## Use roslaunch.core Machine(), make a machine that only has ros_root, pkg_path
    def check_nodes(self, ignore_machine_env = True):
        if self.config is None:
            if not self.quiet:
                print 'Must parse launch file first.'
            return False

        try:
            for node in self.config.nodes:
                paths = _find_node(node.package, node.type)

                if not paths:
                    if not self.quiet:
                        print 'Unable to locate node type %s in package %s' % (node.type, node.package)
                    return False
                if len(paths) > 1:
                    if not self.quiet:
                        print 'Found multiple instances of node type %s in package %s' % (node.type, node.package)
                    return False

            return True
        except:
            if not self.quiet:
                print 'Caught unknown exception parsing launch file'
                traceback.print_exc()
            return False
                
    def check_config_errors(self):
        if self.config is None:
            if not self.quiet:
                print 'Must parse launch file first.'
            return False
        if len(self.config.config_errors) > 0:
            if not self.quiet:
                print 'Configuration errors: %s' % ', '.join(self.config.config_errors)
            return False
        return True
