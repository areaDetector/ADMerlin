#!/bin/env dls-python2.6

from pkg_resources import require
require("cothread")
from cothread.catools import *
import cothread
import time

beginTime = time.time()
count = 100

for i in range(0,count):
    # do an acquire
    start = time.time()
    print "acquiring ..."
    caput("BL16I-EA-DET-12:Merlin1:Acquire", "1", wait=True, timeout = 20)
    finish = time.time()
    elapsed = finish - start
    
    print "%d done in :           %.2f" % (i,elapsed)

totalTime  = time.time() - beginTime
print "completed %d images in %.2f seconds" % (count, totalTime)


