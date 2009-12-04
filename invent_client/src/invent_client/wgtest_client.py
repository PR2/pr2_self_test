#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""

import os, sys, string, time, getopt

import urllib
import invent_client
import neo_cgi, neo_util

import yaml
import types

def obj2hdf(prefix, obj, hdf=None):
  return py2hdf(prefix, obj.__dict__)

def py2hdf(k, v, hdf=None):
  if not hdf: hdf = neo_util.HDF()

  if type(v) == str:
    hdf.setValue(k, v)
  elif type(v) == unicode:
    hdf.setValue(k, str(v))
  elif type(v) in (int, float, long):
    hdf.setValue(k, str(v))
  elif type(v) in (list, tuple):
    n = 0
    for item in v:
      n = n + 1
      py2hdf("%s.%d" % (k,n), item, hdf)
  elif type(v) == dict:
    n = 0
    for _k,_v in sorted(v.iteritems()):
      _k = _k.replace(" ", "_")
      py2hdf(k + "." + _k, _v, hdf)
  elif type(v) == types.InstanceType:
    py2hdf(k, v.__dict__, hdf)
    
  
  return hdf

class Value:
  __slots__ = ("value", "min", "max")
  def __init__(self, value, min, max):
    self.value = value
    self.min = min
    self.max = max

class TestData:
  def __init__(self, testname, timestamp, reference):
    self.testname = testname
    self.reference = reference
    self.timestamp = timestamp

    self.attachment = None
    self.parameters = {}
    self.measurements = {}

  def set_parameter(self, key, value):
    self.parameters[key] = value

  def set_measurement(self, key, value, min, max):
    self.measurements[key] = Value(value, min, max)
    #self.measurements[key] = (value, min, max)

  def set_attachment(self, content_type, fn):
    self.attachment = {"content_type":content_type, "filename":fn}

  def submit(self, client):
    url = client.site + 'wgtest/api.py?Action.Add_TestData=1'

    hdf = obj2hdf("test", self)
    data = hdf.writeString()
    print data

    values = {'data': data}
    data = urllib.urlencode(values)

    body = client.opener.open(url, data).read()

    ohdf = neo_util.HDF()
    ohdf.readString(body)
    
    runid = ohdf.getValue("runid", "")
    status = ohdf.getValue("status", "")

    print runid, status

    if 0:
      attachment = open(self.attachment['filename'], "rb").read()
      client.add_attachment(self.reference, 
                            self.attachment['filename'],
                            self.attachment['content_type'],
                            attachment)

    return True


def test(client):
  import random

#  for k in range(10000):
  #for k in range(1):
  if 1:
    td = TestData("caster/qual/right/hyst", time.time(), "680410801000")
    td.set_parameter('gain 1', 4.2)
    td.set_parameter('gain 2', 8.9)
##    for j in range(random.randint(1,5)):
    if 1:
      j = 1
      td.set_measurement('hystersis %d' % j, random.uniform(1,10), 2., 8.)

    td.set_attachment("text/html", "test.html")

    td.submit(client)

  

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug", "username=", "password="])

  username = os.environ['USER']
  password = None
  testflag = 0

  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugfull()
    elif field == "--test":
      testflag = 1
    elif field == "--username":
      username = val
    elif field == "--password":
      password = val

  if testflag:
    test()
    return

  client = invent_client.Invent(username, password)
  if client.login() == False:
    print "cannot login"
    return
  test(client)


if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
