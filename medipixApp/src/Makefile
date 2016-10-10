TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY += medipixDetector

PROD_Linux += medipix_sim

#build cpp with debug
USR_CFLAGS += -DDEBUG
HOST_OPT=NO

# ------------------------
# Build Test Executables
# ------------------------

# Add locally compiled object code
medipix_sim_SRCS_Linux += medipix_sim.c
medipix_sim_SYS_LIBS_Linux += pthread

# ------------------------
# Build the Area Detector Derived Library
# ------------------------

DBD += medipix.dbd

medipixDetector_SRCS += medipixDetector.cpp
medipixDetector_SRCS += mpxConnection.cpp

include $(ADCORE)/ADApp/commonLibraryMakefile

include $(TOP)/configure/RULES
