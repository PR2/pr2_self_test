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

##\author Kevin Watts

import roslib; roslib.load_manifest('qualification')

import os, sys
from xml.dom import minidom

TESTS_DIR = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'tests')
CONFIG_DIR = os.path.join(roslib.packages.get_pkg_dir('qualification'), 'config')

def load_tests_from_map(tests, test_descripts_by_file):
  # Load test directory
  tests_xml_path = os.path.join(TESTS_DIR, 'tests.xml')
  try:
    doc = minidom.parse(tests_xml_path)
  except IOError:
    import traceback
    traceback.print_exc()
    print >> sys.stderr, "Could not load tests description from '%s'"%(tests_xml_path)
    return False  
  
  # Loads tests by serial number of part
  test_elements = doc.getElementsByTagName('test')
  for test in test_elements:
    serial = test.attributes['serial'].value
    test_file = test.attributes['file'].value
    descrip = test.attributes['descrip'].value
    if tests.has_key(serial):
      tests[serial].append(test_file)
    else:
      tests[serial] = [ test_file ]
      
    test_descripts_by_file[test_file] = descrip
    
  return True

def load_configs_from_map(config_files, config_descripts_by_file):
  # Load part configuration scripts
  config_xml_path = os.path.join(CONFIG_DIR, 'configs.xml')
  try:
    doc = minidom.parse(config_xml_path)
  except IOError:
    print >> sys.stderr, "Could not load configuation scripts from '%s'"%(config_xml_path)
    return False
    
  config_elements = doc.getElementsByTagName('config')
  for conf in config_elements:
    try:
      serial = conf.attributes['serial'].value
      file = conf.attributes['file'].value
      descrip = conf.attributes['descrip'].value
      
      powerboard = True
      if conf.attributes.has_key('powerboard'):
        powerboard = conf.attributes['powerboard'].value != "false"
    except:
      print 'Caught exception parsing configuration file'
      import traceback
      traceback.print_exc()
      return False

    # Generate test XML. If we need power board, add prestartup/shutdown
    # to turn on/off power
    test = ['<test name="%s">' % descrip]
    if powerboard:
      test.append('<pre_startup name="Power Cycle">scripts/power_cycle.launch</pre_startup>')
    test.append('<pre_startup name="%s">config/%s</pre_startup>' % (descrip, file))
    test.append('<subtest name="%s Test">config/subtest_conf.launch</subtest>' % (descrip))
    if powerboard:
      test.append('<shutdown name="Shutdown">scripts/power_board_disable.launch</shutdown>')
    test.append('</test>')
      
    test_str = '\n'.join(test)

    if config_files.has_key(serial):
      config_files[serial].append(test_str)
    else:
      config_files[serial] = [ test_str ]
      
    config_descripts_by_file[ test_str ] = descrip

  return True
