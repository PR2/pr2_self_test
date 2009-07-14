#!/usr/bin/env python
#
# hdfhelp.py
#

import string, os
import neo_util

def loopHDF(hdf, name=None):
  results = []
  if name: o = hdf.getObj(name)
  else: o = hdf
  if o:
    o = o.child()
    while o:
      results.append(o)
      o = o.next()
  return results


def loopKVHDF(hdf, name=None):
  results = []
  if name: o = hdf.getObj(name)
  else: o = hdf
  if o:
    o = o.child()
    while o:
      results.append((o.name(), o.value()))
      o = o.next()
  return results


class hdf_iterator:
  def __init__(self, hdf):
    self.hdf = hdf
    self.node = None
    if self.hdf:
      self.node = self.hdf.child()

  def __iter__(self): return self

  def next(self):
    if not self.node:
      raise StopIteration

    ret = self.node
    self.node = self.node.next()
      
    return ret

class hdf_kv_iterator(hdf_iterator):
  def next(self):
    if not self.node: raise StopIteration

    ret = (self.node.name(), self.node.value())
    self.node = self.node.next()
      
    return ret

class hdf_key_iterator(hdf_iterator):
  def next(self):
    if not self.node: raise StopIteration

    ret = self.node.name()
    self.node = self.node.next()
      
    return ret

class hdf_ko_iterator(hdf_iterator):
  def next(self):
    if not self.node: raise StopIteration

    ret = (self.node.name(), self.node)
    self.node = self.node.next()
      
    return ret
  

def hdfExportDict(prefix, hdf, dict):
  for k,v in dict.items():
    hdf.setValue(prefix + "." + str(k), str(v))


def hdfExportList(prefix, hdf, list):
  n = 0
  for item in list:
    n = n + 1
    hdf.setValue(prefix + "." + str(n), str(item))







