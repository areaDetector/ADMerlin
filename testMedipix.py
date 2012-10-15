#!/bin/env dls-python2.6

from pkg_resources import require
from epics import caput
require("cothread")
from cothread.catools import *
import cothread
import time


for i in range(1,1000):
    # do an acquire
    start = time.time()
    print "acquiring ..."
    caput("Medipix1Test3Acquire", "1", wait=True)
    finish = time.time()
    elapsed = finish - start
    
    print "done in :           %.2f" % elapsed


