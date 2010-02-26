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

import simple_hdfhelp as hdfhelp

def obj2hdf(prefix, obj, hdf=None):
  return py2hdf(prefix, obj.__dict__)

def py2hdf(k, v, hdf=None):
  if not hdf: hdf = neo_util.HDF()

  k.replace('.', '')
  #if k[-1] == ".":
  #  k[-1] = ''

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

    self.note = None

  def set_parameter(self, key, value):
    self.parameters[key] = value

  def set_measurement(self, key, value, min, max):
    self.measurements[key] = Value(value, min, max)

  def set_attachment(self, content_type, fn):
    self.attachment = {"content_type":content_type, "filename":fn}

  def set_note(self, note):
    self.note = note

  def submit(self, client):
    if not client.login():
      return False

    url = client.site + 'wgtest/api.py?Action.Add_TestData=1'

    hdf = obj2hdf("test", self)
    test_data = hdf.writeString() 

    values = {'data': test_data}
    data = urllib.urlencode(values)

    body = client.opener.open(url, data).read()

    try:    
      ohdf = neo_util.HDF()
      ohdf.readString(body)
    
      runid = ohdf.getValue("runid", "")
      status = ohdf.getValue("status", "")
    except:
      print body

      return False

    if self.attachment and False:
      attachment = open(self.attachment['filename'], "rb").read()
      client.add_attachment(self.reference, 
                            self.attachment['filename'],
                            self.attachment['content_type'],
                            attachment)

    #print body

    return status == '1'

# debug only
def get_test_runs(key, client):
  if not client.login():
    return {}

  key = key.strip()

  url = client.site + 'wgtest/api.py?Action.Get_TestRuns=1&key=%s' % (key,)
  
  body = client.opener.open(url).read()

  #print body
  
  hdf = neo_util.HDF()
  hdf.readString(body)
  
  rv = {}
  for k, o in hdfhelp.hdf_ko_iterator(hdf.getObj("CGI.cur.test")):
    rv[o.getValue("runid", "")] = o.getValue("testid", "")

  return rv



# debug only
def get_run_data(runid, client):
  if not client.login():
    return None

  runid = runid.strip()

  url = client.site + 'wgtest/api.py?Action.Get_RunData=1&runid=%s' % (runid,)
  
  body = client.opener.open(url).read()

  #print body
  
  ohdf = neo_util.HDF()
  ohdf.readString(body)

  testid = ohdf.getValue("testid", "")
  reference = ohdf.getValue("reference", "")
  timestamp = ohdf.getValue("timestamp", "")

  rv = []
  for k, o in hdfhelp.hdf_ko_iterator(ohdf.getObj("values")):
    rv.append((o.getValue("key", ""), o.getValue("value", ""), o.getValue("min", ""), o.getValue("max", "")))

  return rv


#debug only
def get_test_data(testid, value, client):
  if not client.login():
    return None

  testid = testid.strip()

  value = value.strip().replace(' ', '_')

  url = client.site + 'wgtest/api.py?Action.Get_TestData=1&testid=%s&value=%s' % (testid, value)
  
  body = client.opener.open(url).read()

  #print body
  
  ohdf = neo_util.HDF()
  ohdf.readString(body)
  
  if ohdf.getValue("key", "Invalid") == "Invalid":
    return []

  rv = []
  for k, o in hdfhelp.hdf_ko_iterator(ohdf.getObj("values")):
    rv.append((o.getValue("runid", ""), o.getValue("timestamp", ""), o.getValue("value", ""), o.getValue("min", ""), o.getValue("max", "")))

  return rv

# debug only
def get_tests_by_name(test_name, client):
  if not client.login():
    return None

  test_name = test_name.strip()

  value = value.strip().replace(' ', '_')

  url = client.site + 'wgtest/api.py?Action.Get_TestsByName=1&test_name=%s&value=%s' % (testid, value)
  
  body = client.opener.open(url).read()

  #print body
  
  ohdf = neo_util.HDF()
  ohdf.readString(body)
