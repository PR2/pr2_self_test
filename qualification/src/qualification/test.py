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

##\author Kevin Watts, Josh Faust

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
  def __init__(self, subtest, key, name, timeout = -1, pre_test=None, post_test=None):
    self._test_script = subtest
    self._pre_script = pre_test
    self._post_script = post_test
    self._name = name
    self._key = key
    self._timeout = timeout

  ##\brief Check that key exists, launch files exist and are ".launch" files
  def validate(self):
    if self._name is None:
      print 'Subtests must be named.'
      return False

    if self._key is None:
      print 'Subtest key is none.'
      return False

    if self._test_script is None:
      print 'Subtest has not launch script'
      return False

    if not os.path.exists(self._test_script):
      print 'Subtest launch file does not exist: %s' % self._test_script
      return False

    if not self._test_script.endswith(".launch"):
      print 'Given test script is not a launch file: %s' % self._test_script
      return False
    
    if self._pre_script is not None:
      if not os.path.exists(self._pre_script):
        print 'Subtest pre-launch file is listed, but does not exist: %s' % self._pre_script
        return False
      if not self._pre_script.endswith(".launch"):
        print 'Given pre launch script is not a launch file: %s' % self._pre_script
        return False
      
    if self._post_script is not None:
      if not os.path.exists(self._post_script):
        print 'Subtest post-launch file is listed, but does not exist: %s' % self._post_script
        return False
      if not self._post_script.endswith(".launch"):
        print 'Given post launch script is not a launch file: %s' % self._post_script
        return False

    return True
  

  ## Returns subtest key to allow subtests to load in order
  def get_key(self):
    return self._key

  def get_timeout(self):
    return self._timeout

  def get_name(self):
    if self._name is not None:
      return self._name

    return '%s: %d' % (os.path.basename(self._test_script), self._key)


##\brief Holds pre-startup/shutdown scripts for qual tests
##\todo Change private vars to _names
class TestScript:
  ##@param launch_file: Complete file name of launch file
  ##@param name str (optional): Human readable name of pre-startup script
  ##@param timeout int (optional) : Timeout in seconds of test
  def __init__(self, launch_file, name=None, timeout = -1):
    self.launch_file = launch_file
    self.name = name
    self.timeout = timeout
  
  ##\brief Check that launch file exists and is ".launch" file
  def validate(self):
    if self.name is None:
      print 'TestScripts must be named'
      return False

    if self.launch_file is None:
      print 'No launch file found, invalid TestScript'
      return False

    if not os.path.exists(self.launch_file):
      print 'TestScript launch file %s does not exist' % self.launch_file
      return False

    if not self.launch_file.endswith(".launch"):
      print 'TestScript launch file %s is not a .launch file' % self.launch_file
      return False

    return True

  def get_timeout(self):
    return self.timeout
    
  ## Returns name, or launch filename if no name
  def get_name(self):
    if self.name is not None:
      return self.name

    return os.path.basename(self.launch_file)
      
##\brief Qualification test to run. 
##
##Holds instructions, subtests, pre_startup scripts, etc.
class Test:
  def __init__(self):
    self._name = None
    self._startup_script = None
    self._shutdown_script = None
    self._instructions_file = None
    self.pre_startup_scripts = []
    self.subtests = []

  ##\brief Check that all prestartups, subtests, instructions, etc are real files
  ##
  ## This does not do any kind of parsing of the files themselves. It only
  ## checks that they exist, and that all files are ".launch" files. Instructions
  ## files must be ".html" files. Checks that test is named.
  ##\return True if Test is valid.
  def validate(self):
    if self._name is None:
      print 'Qualification tests must be named.'
      return False

    if len(self.subtests) == 0 and len(self.pre_startup_scripts) == 0:
      print 'No subtests or prestartup scripts. Not loaded'
      return False

    if self._startup_script is not None and not self._startup_script.validate():
      print 'Startup script does not exist: %s' % self._startup_script.launch_file
      return False

    if self._shutdown_script is not None and not self._shutdown_script.validate():
      print 'Shutdown script does not exist: %s' % self._shutdown_script.launch_file
      return False

    if self._instructions_file is not None:
      if not os.path.exists(self._instructions_file):
        print 'Instructions file does not exist: %s' % self._instructions_file
        return False
      if not self._instructions_file.endswith('.html'):
        print 'Instructions file %s is not a ".html" file' % self._instructions_file
        return False

    for prestart in self.pre_startup_scripts:
      if not prestart.validate():
        return False

    for subtest in self.subtests:
      if not subtest.validate():
        return False

    return True
    
     
  ##\brief Loads qual test from and XML string
  ##@param test_str: XML file to load, as string
  ##@param test_dir: Base directory of test, appended to qual tests
  ##@return True if load succeeded
  def load(self, test_str, test_dir):
    try:
      doc = minidom.parseString(test_str)
    except IOError:
      rospy.logerr('Unable to parse test string:\n%s' % test_str)
      return False
    
    test_list = doc.getElementsByTagName('test')
    if len(test_list) != 1:
      return False
    test_main = test_list[0]
    if not test_main.attributes.has_key('name'):
      return False
    self._name = test_main.attributes['name'].value

    pre_startups = doc.getElementsByTagName('pre_startup')
    if (pre_startups != None and len(pre_startups) > 0):
      for pre_startup in pre_startups:
        launch = os.path.join(test_dir, pre_startup.childNodes[0].nodeValue)
        name = None
        timeout = -1

        if not pre_startup.attributes.has_key('name'):
          print 'Prestartup script missing name.'
          return False
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
      else:
        name = elems[0].childNodes[0].nodeValue
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
        timeout = -1
        
        if not st.attributes.has_key('name'):
          return False

        if (st.attributes.has_key('post')):
          post = os.path.join(test_dir, st.attributes['post'].value)
        if (st.attributes.has_key('pre')):
          pre = os.path.join(test_dir, st.attributes['pre'].value)
        if st.attributes.has_key('timeout'):
          timeout = int(st.attributes['timeout'].value)

        name = st.attributes['name'].value
        
        key = key_count
        key_count += 1
        self.subtests.append(SubTest(script, key, name, timeout, pre, post))

    return True
                                        
  ##\brief Makes a startup script using a pkg and a launch file
  ##@param pkg: Package of launch file
  ##@param launch_file: Filepath 
  def generate_onboard_test(self, pkg, launch_file, name):
    file = os.path.join(roslib.packages.get_pkg_dir(pkg), launch_file)
    self._startup_script = TestScript(file, name)
                                        
  ##\brief Adds subtests, eliminates duplicates and sorts by keys
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

  ##\todo Change functions names to Python style
  ##\brief Name or basename or startup script
  def getName(self):
    return self._name
 
  def get_name(self):
    return self._name
    
  ##\brief Full path to startup script
  def getStartupScript(self):
    return self._startup_script
                                
  ##\brief Full path to shutdown script
  def getShutdownScript(self):
    return self._shutdown_script
  
  ##\brief Full path to instructions file
  def getInstructionsFile(self):
    return self._instructions_file
  
  
