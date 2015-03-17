#!/bin/env dls-python

from pkg_resources import require
from epics import caput
require("numpy")
require("cothread")
from cothread.catools import *
import cothread
import time
import sys

beginTime = time.time()
count = 5

caput("Medipix1:HDF5:FileWriteMode", "Stream")
caput("Medipix1:HDF5:FilePath","/tmp/lancelot",datatype=DBR_CHAR_STR)
caput("Medipix1:HDF5:FileName","image",datatype=DBR_CHAR_STR)
caput("Medipix1:HDF5:FileTemplate","%s%s_%d.h5",datatype=DBR_CHAR_STR)
caput("Medipix1:HDF5:NDArrayPort","MPX1")
#caput("",)
#caput("",)
#caput("",)
#caput("",)

caput("Medipix1:HDF5:Capture", 1)

for i in range(0,count):
    # do an acquire
    start = time.time()
    print "acquiring ..."
    caput("Medipix1:XBPM:Acquire", "1", wait=True, timeout = 20)
    finish = time.time()
    elapsed = finish - start
    
    print "%d done in :           %.2f" % (i,elapsed)

totalTime  = time.time() - beginTime
print "completed %d images in %.2f seconds" % (count, totalTime)

totalTimeWrite  = time.time() - beginTime
caput("Medipix1:HDF5:Capture", 0, wait=True, timeout = 20)
print "completed writing HDF5 in %.2f" % (totalTimeWrite - totalTime)



