#! /usr/bin/env python

"""
usage: %(progname)s [args]
"""


import os, sys, string, time, getopt

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
    self.measurements[key] = (value, min, max)

  def set_attachment(self, content_type, attachment):
    self.attachment = (content_type, attachment)

  def submit(self):
    return True


def test():
  td = TestData("caster/qual/right/hyst", time.time(), "680410801000")
  td.set_parameter('gain', 4)
  td.set_measurement('hystersis', 5, 2, 8)
  td.submit()
  

def usage(progname):
  print __doc__ % vars()

def main(argv, stdout, environ):
  progname = argv[0]
  optlist, args = getopt.getopt(argv[1:], "", ["help", "test", "debug"])

  testflag = 0

  for (field, val) in optlist:
    if field == "--help":
      usage(progname)
      return
    elif field == "--debug":
      debugfull()
    elif field == "--test":
      testflag = 1

  if testflag:
    test()
    return


if __name__ == "__main__":
  main(sys.argv, sys.stdout, os.environ)
