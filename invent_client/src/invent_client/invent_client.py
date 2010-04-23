#! /usr/bin/env python
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

##\author Scott Hassen
##\brief Client for WG inventory system

PKG = 'invent_client'

import os, sys, string, time, getopt, re

import urllib2, cookielib

import mimetypes
import mimetools

import neo_cgi, neo_util
import simple_hdfhelp as hdfhelp

import roslib; roslib.load_manifest(PKG)

##\brief Checks if given serial number is a valid WG SN
def _is_serial_valid(reference):
  if len(reference) != 12:
    return False

  if not reference.startswith('680'):
    return False

  return True


## \brief Stores username and password, provides access to invent
##
## Performs all action relating to inventory system
## Will login automatically before all functions if needed
class Invent:
  ##@param username str : Username for WG invent system
  ##@param password str : Password for WG invent system
  def __init__(self, username, password, debug=False):
    self.username = username
    self.password = password

    self.cj = cookielib.CookieJar()
    self.opener = urllib2.build_opener(urllib2.HTTPCookieProcessor(self.cj))

    self.loggedin = False
    self._logged_time = 0

    self.site = "http://invent.willowgarage.com/invent/"
    if debug:
      self.site = "http://cmi.willowgarage.com/invent/"

  ##\brief Logs into Invent. Returns true if successful
  def login(self):
    dt = time.time() - self._logged_time
    if self.loggedin==False or dt > 3600:
      return self._login()
    return True

  ## Login to inventory system with given username and password
  ## Returns True if successful, False if failure
  def _login(self):
    username = self.username
    password = self.password
    url = self.site + "login/signin0.py?Action.Login=1&username=%(username)s&password=%(password)s" % locals()

    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    self.loggedin = False
    self._logged_time = 0
    if body.find("Invalid Login") != -1:
      return False

    self.loggedin = True
    self._logged_time = time.time()
    return True

  ##\brief Debug mode only. Check invent DB for serial
  ##
  ##\return bool : True if serial is valid, False if not
  def check_serial_valid(self, serial):
    if not _is_serial_valid(serial):
      return False

    url = self.site + "invent/api.py?Action.isItemValid=1&reference=%s" % (serial,)
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    i = string.find(body, "\n<!--")
    value = string.strip(body[:i])

    if value != "True":
      return False

    ##\todo Need some other stuff here. #4060
    return True

  ##\brief Debug mode only
  def get_attachments(self, key):
    self.login()

    key = key.strip()

    url = self.site + "invent/api.py?Action.getAttachments=1&key=%s" % (key,)
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    print body

    hdf = neo_util.HDF()
    hdf.readString(body)

    ret = {}
    for k,o in hdfhelp.hdf_ko_iterator(hdf.getObj("CGI.cur.attachments")):
      ret[o.getValue("aid", "")] = o.getValue("name", "")
    
    return ret
    

  ## Return any references to an item. References are grouped by
  ## name, and are stored as NAME:REFERENCE,... under each item.
  ##@param key str : Serial number of item
  ##@return Dictionary of { name, reference }
  def getItemReferences(self, key):
    self.login()

    key = key.strip()

    url = self.site + "invent/api.py?Action.getItemReferences=1&key=%s" % (key,)
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    hdf.readString(body)

    ret = {}
    for k,o in hdfhelp.hdf_ko_iterator(hdf.getObj("CGI.cur.refs")):
      ret[o.getValue("name", "")] = o.getValue("reference", "")
    
    return ret

  ## Add reference to an item
  ##@param key str : Serial number of item
  ##@param name str : Reference name
  ##@param reference str : Reference value
  def addItemReference(self, key, name, reference):
    self.login()

    key = key.strip()

    url = self.site + "invent/api.py?Action.addItemReference=1&key=%s&name=%s&reference=%s" % (key,urllib2.quote(name),urllib2.quote(reference))
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()
    
  ## Generates Willow Garage mac address for item. Used for forearm cameras
  ## Does not return mac address
  ##@param key str : Serial number of item
  ##@param name str : Interface name (ex: "lan0")
  def generateWGMacaddr(self, key, name):
    self.login()

    key = key.strip()

    url = self.site + "invent/api.py?Action.generateWGMacaddr=1&key=%s&name=%s" % (key,urllib2.quote(name))
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

  ##\brief Sets notes of component
  ## Sets a note value for the component. Allows users to set the text of 
  ## a particular note if the noteid parameter is specified. Returns
  ## noteid to allow note edits. Returns None if error.
  ##@param reference str : Serial number of item
  ##@param note str : Text of item
  ##@param noteid int (optional) : Note ID, allows programmatic access to note text
  def setNote(self, reference, note, noteid = None):
    self.login()

    url = self.site + "invent/api.py?Action.AddNoteToItem=1&reference=%s&note=%s" % (reference, urllib2.quote(note))
    if noteid:
      url = url + "&noteid=%s" % noteid

    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    pat = re.compile("rowid=([0-9]+)")
    m = pat.search(body)
    if m:
      noteid = int(m.group(1))
      return noteid
    return None

  ##\brief Set value of component's key
  ## Set key-value of component. Ex: setKV(my_ref, 'Test Status', 'PASS')
  ##@param reference str : Serial number of component
  ##@param key str : Key (name)
  ##@param value str : Value
  def setKV(self, reference, key, value):
    self.login()

    key = key.strip()
    value = value.strip()

    if not key:
      raise ValueError, "the key is blank"
    if not value:
      raise ValueError, "the value is blank"

    url = self.site + "invent/api.py?Action.setKeyValue=1&reference=%s&key=%s&value=%s" % (reference, urllib2.quote(key), urllib2.quote(value))

    fp = self.opener.open(url)
    fp.read()
    fp.close()

  ##\brief Returns True if part has 'Test Status'='PASS', False otherwise
  def get_test_status(self, reference):
    return self.getKV(reference, 'Test Status') == 'PASS'

  ##\brief Return value of component's key
  ## Get key-value of component. Ex: 'PASS' = getKV(my_ref, 'Test Status')
  ##@param reference str : Serial number of component
  ##@param key str : Key (name)
  def getKV(self, reference, key):
    self.login()

    key = key.strip()

    if not key:
      raise ValueError, "the key is blank"

    url = self.site + "invent/api.py?Action.getKeyValue=1&reference=%s&key=%s" % (reference, urllib2.quote(key))

    fp = self.opener.open(url)
    value = fp.read()
    fp.close()

    i = string.find(value, "\n<!--")
    value = string.strip(value[:i])
    
    return value

  ##\brief Adds attachment to component
  ##
  ## Adds file as attachment to component. Attachment it encoded to unicode,
  ## then uploaded. Mimetype is optional, but helps users view
  ## attachments in window. 
  ##@param reference str : Serial number of component
  ##@param name str : Attachment filename
  ##@param mimetype MIMEType : MIMEType of file
  ##@param attachment any : Attachement data
  def add_attachment(self, reference, name, mimetype, attachment, note = None, id=None):
    self.login()

    if not name:
      raise ValueError, "the name is blank"

    theURL = self.site + "invent/api.py"

    fields = []
    fields.append(('Action.addAttachment', "1"))
    fields.append(('reference', reference))
    fields.append(('mimetype', mimetype))
    fields.append(('name', name))
    if note is not None:
      fields.append(('note', note))
    if id is not None:
      fields.append(('aid', id))

    files = []
    files.append(("attach", name, attachment))

    input = build_request(theURL, fields, files)

    response = self.opener.open(input).read()

    pat = re.compile("rowid=([0-9]+)")
    m = pat.search(response)
    if m is not None:
      id = int(m.group(1))
      return id
    return None

  ##\brief Returns list of sub items (references) for a particular parent
  ##\param reference str : WG PN of component or assembly
  ##\param recursive bool [optional] : Sub-sub-...-sub-parts of reference
  def get_sub_items(self, reference, recursive = False):
    self.login()
    
    reference = reference.strip()

    url = self.site + "invent/api.py?Action.getSubparts=1&reference=%s" % (reference)
    fp = self.opener.open(url)
    body = fp.read()
    fp.close()

    hdf = neo_util.HDF()
    hdf.readString(body)

    ret = []
    for k,o in hdfhelp.hdf_ko_iterator(hdf.getObj("CGI.cur.items")):
      ret.append(o.getValue("reference", ""))
    
    if recursive:
      for rt in ret:
        ret.extend(self.get_sub_items(rt, True))

    return ret

  
      


## -------------------------------------------------------------

def build_request(theurl, fields, files, txheaders=None):
  content_type, body = encode_multipart_formdata(fields, files)
  if not txheaders: txheaders = {}
  txheaders['Content-type'] = content_type
  txheaders['Content-length'] = str(len(body))
  return urllib2.Request(theurl, body, txheaders)

def encode_multipart_formdata(fields, files, BOUNDARY = '-----'+mimetools.choose_boundary()+'-----'):

    """ Encodes fields and files for uploading.

    fields is a sequence of (name, value) elements for regular form fields - or a dictionary.

    files is a sequence of (name, filename, value) elements for data to be uploaded as files.

    Return (content_type, body) ready for urllib2.Request instance

    You can optionally pass in a boundary string to use or we'll let mimetools provide one.

    """    

    CRLF = '\r\n'

    L = []

    if isinstance(fields, dict):
        fields = fields.items()

    for (key, value) in fields:
        L.append('--' + BOUNDARY)
        L.append('Content-Disposition: form-data; name="%s"' % key)
        L.append('')
        L.append(value)

    for (key, filename, value) in files:
        #encoded = value.encode('base64')
        encoded = value
        filetype = mimetypes.guess_type(filename)[0] or 'application/octet-stream'
        L.append('--' + BOUNDARY)
        L.append('Content-Disposition: form-data; name="%s"; filename="%s"' % (key, filename))
        L.append('Content-Length: %s' % len(encoded))
        L.append('Content-Type: %s' % filetype)
        L.append('Content-Transfer-Encoding: binary')
        L.append('')
        L.append(encoded)

    L.append('--' + BOUNDARY + '--')
    L.append('')
    body = CRLF.join([str(l) for l in L])

    content_type = 'multipart/form-data; boundary=%s' % BOUNDARY        # XXX what if no files are encoded

    return content_type, body


