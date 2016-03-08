TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

#LIBRARY += medipix_low
LIBRARY += medipixDetector

PROD += medipix_sim
#PROD += medipix_test

#build cpp with debug
USR_CFLAGS += -DDEBUG
HOST_OPT=NO

# The following are compiled and added to the support library
medipix_low_SRCS += medipix_low.c

# ------------------------
# Build Test Executables
# ------------------------

# Add locally compiled object code
medipix_sim_SRCS += medipix_sim.c
medipix_sim_LDFLAGS += -lpthread

medipix_test_SRCS += medipix_test.c
medipix_test_LIBS += medipix_low

# ------------------------
# Build the Area Detector Derived Library
# ------------------------


USR_INCLUDES += -I$(AREA_DETECTOR)/ADApp/cbfSrc

DBD += medipix.dbd

medipixDetector_SRCS += medipixDetector.cpp
medipixDetector_SRCS += mpxConnection.cpp

medipixDetector_LIBS += cbfad


include $(TOP)/configure/RULES
