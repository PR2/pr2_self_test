#!/usr/bin/env python
#
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

# Author: Kevin Watts

import roslib
roslib.load_manifest('life_test')


class FailedLoadError(Exception): pass
class BayNameExistsError(Exception): pass

##\brief Collection of test bays
class TestRoom:
    def __init__(self, hostname):
        self.hostname = hostname
        self._bays = {}

    def add_bay(self, bay):
        if self._bays.has_key(bay.name):
            raise BayNameExistsError
        self._bays[bay.name] = bay

    def get_bay_names(self, need_power):
        if not need_power:
            bays = self._bays.keys()
            bays.sort()
            return bays

        names = []
        for name in self._bays.keys():
            if self._bays[name].board is not None:
                names.append(name)
        names.sort()
        return names

    def get_bay(self, name):
        if not self._bays.has_key(name):
            return None
        return self._bays[name]

##\brief Computer, powerboard and breaker to run test
class TestBay:
    def __init__(self, xml_doc):
        self.name = xml_doc.attributes['name'].value
        self.machine = xml_doc.attributes['machine'].value
        if xml_doc.attributes.has_key('board'):
            self.board = int(xml_doc.attributes['board'].value)
            self.breaker = int(xml_doc.attributes['breaker'].value)
            if self.breaker not in [0, 1, 2]:
                raise FailedLoadError
        else:
            self.board = None
            self.breaker = None
        




                            
                            
