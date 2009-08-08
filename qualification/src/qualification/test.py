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

# Author: Kevin Watts

PKG = 'qualification'

import os
import roslib
roslib.load_manifest(PKG)

from xml.dom import minidom

class NotADirectoryError(Exception): pass
class TestDoesNotExistError(Exception): pass
class FailedLoadError(Exception): pass

## Qualification Subtest. Each subtest has a main launch file and may have 
## pre- and post- startup scripts. All launch files are stored with their
## complete paths
##@param subtest str: Launch file of subtest
##@param key int : Allows the subtests to load in order, lowest key first
##@param pre_test str (optional): Launch file for pre-subtest script
##@param post_test str (optional): Launch file for post-subtest script
##@param name str (optional): Human readable name of subtest
class SubTest:
  def __init__(self, subtest, key, name=None, timeout = -1, pre_test=None, post_test=None):
    self._test_script = subtest
    self._pre_script = pre_test
    self._post_script = post_test
    self._name = name
    self._key = key
    self._timeout = timeout

  ## Returns subtest key to allow subtests to load in order
  def get_key(self):
    return self._key

  def get_timeout(self):
    return self._timeout

  ## Returns name of test, or generates one if none exists
  def get_name(self):
    if self._name:
      return self._name

    if not self._key:
      return os.path.basename(self._test_script)
      # If key is none, generate name from pre, post, test scripts
    return '%s%d' % (os.path.basename(self._test_script), self._key)


## Holds pre-startup/shutdown scripts for qual tests
class TestScript:
  ##@param launch_file: Complete file name of launch file
  ##@param name str (optional): Human readable name of pre-startup script
  ##@param timeout int (optional) : Timeout in seconds of test
  def __init__(self, launch_file, name=None, timeout = -1):
    self.launch_file = launch_file
    self.name = name
    self.timeout = timeout
  
  def get_timeout(self):
    return self.timeout
    
  ## Returns name, or launch filename if no name
  def get_name(self):
    if self.name:
      return self.name

    return os.path.basename(self.launch_file)
      
## Qualification test to run. Holds instructions, subtests, pre_startup
## scripts, etc.
class Test:
  def __init__(self):
    self._name = None
    self._startup_script = None
    self._shutdown_script = None
    self._instructions_file = None
    self.pre_startup_scripts = []
    self.subtests = []
    
  ## Loads qual test from and XML string
  ##@param test_str: XML file to load, as string
  ##@param test_dir: Base directory of test, appended to qual tests
  def load(self, test_str, test_dir):
    try:
      self._doc = minidom.parseString(test_str)
    except IOError:
      raise FailedLoadError
    
    doc = self._doc

    elems = doc.getElementsByTagName('name')
    if (elems != None and len(elems) > 0):
      self._name = elems[0].childNodes[0].nodeValue
    
    pre_startups = doc.getElementsByTagName('pre_startup')
    if (pre_startups != None and len(pre_startups) > 0):
      for pre_startup in pre_startups:
        launch = os.path.join(test_dir, pre_startup.childNodes[0].nodeValue)
        name = None
        timeout = -1

        if (pre_startup.attributes.has_key('name')):
          name = pre_startup.attributes['name'].value              

        if (pre_startup.attributes.has_key('timeout')):
          timeout = int(pre_startup.attributes['timeout'].value)           
          
        self.pre_startup_scripts.append(TestScript(launch, name, timeout))
        
    elems = doc.getElementsByTagName('startup')
    if (elems != None and len(elems) > 0):
      launch = os.path.join(test_dir, elems[0].childNodes[0].nodeValue)
      name = None
      if (elems[0].attributes.has_key('name')):
        name = elems[0].attributes['name'].value  
      self._startup_script = TestScript(launch, name)
    
    elems = doc.getElementsByTagName('shutdown')
    if (elems != None and len(elems) > 0):
      launch = os.path.join(test_dir, elems[0].childNodes[0].nodeValue)
      name = None
      timeout = -1

      if (elems[0].attributes.has_key('name')):
        name = elems[0].attributes['name'].value  

      if (elems[0].attributes.has_key('timeout')):
        timeout = int(pre_startup.attributes['timeout'].value)           

      self._shutdown_script = TestScript(launch, name, timeout)
                                        
    elems = doc.getElementsByTagName('instructions')
    if (elems != None and len(elems) > 0):
      self._instructions_file = os.path.join(test_dir, elems[0].childNodes[0].nodeValue)
    
    key_count = 1
    subtests = doc.getElementsByTagName('subtest')
    if (subtests != None and len(subtests) > 0):
      for st in subtests:
        script = os.path.join(test_dir, st.childNodes[0].nodeValue)
        pre = None
        post = None
        name = None
        timeout = -1
        
        if (st.attributes.has_key('post')):
          post = os.path.join(test_dir, st.attributes['post'].value)
        if (st.attributes.has_key('pre')):
          pre = os.path.join(test_dir, st.attributes['pre'].value)
        if (st.attributes.has_key('name')):
          name = st.attributes['name'].value
        if (st.attributes.has_key('timeout')):
          timeout = int(st.attributes['timeout'].value)
        
        key = key_count
        key_count += 1
        self.subtests.append(SubTest(script, key, name, timeout, pre, post))
                                        
  ## Makes a startup script using a pkg and a launch file
  ##@param pkg: Package of launch file
  ##@param launch_file: Filepath 
  def generate_onboard_test(self, pkg, launch_file, name):
    file = os.path.join(roslib.packages.get_pkg_dir(pkg), launch_file)
    self._startup_script = TestScript(file, name)
                                        
  ## Adds subtests, eliminates duplicates and sorts by keys
  ##@param subtests: List of SubTest's
  def add_subtests(self, subtests):
    self.subtests.extend(subtests)

    st_set = set(self.subtests)

    st_dict = {}
    for st in st_set:
      st_dict[st.get_key()] = st

    keys = st_dict.keys()
    keys.sort()

    self.subtests = []
    for key in keys:
      if st_dict[key]:
        self.subtests.append(st_dict[key])

  ## Name or basename or startup script
  def getName(self):
    if self._name:
      return self._name
    return os.path.basename(self._startup_script)

  ## Full path to startup script
  def getStartupScript(self):
    return self._startup_script
                                
  ## Full path to shutdown script
  def getShutdownScript(self):
    return self._shutdown_script
  
  ## Full path to instructions file
  def getInstructionsFile(self):
    return self._instructions_file
  
  
