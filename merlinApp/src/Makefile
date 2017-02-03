TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

#LIBRARY += merlin_low
LIBRARY += merlinDetector

PROD_Linux += merlin_sim
#PROD += merlin_test

#build cpp with debug
USR_CFLAGS += -DDEBUG
HOST_OPT=NO

# The following are compiled and added to the support library
merlin_low_SRCS += merlin_low.c

# ------------------------
# Build Test Executables
# ------------------------

# Add locally compiled object code
merlin_sim_SRCS += merlin_sim.c
merlin_sim_SYS_LIBS += pthread

merlin_test_SRCS += merlin_test.c
merlin_test_LIBS += merlin_low

# ------------------------
# Build the Area Detector Derived Library
# ------------------------

DBD += merlin.dbd

merlinDetector_SRCS += merlinDetector.cpp
merlinDetector_SRCS += mpxConnection.cpp

include $(ADCORE)/ADApp/commonLibraryMakefile

include $(TOP)/configure/RULES
