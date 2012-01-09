TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY += medipix_low

# The following are compiled and added to the support library
medipix_low_SRCS += medipix_low.c


# ------------------------
# Build Test Executables
# ------------------------

PROD = medipix_sim
PROD += medipix_test

# Add locally compiled object code
medipix_sim_SRCS += medipix_sim.c
medipix_test_SRCS += medipix_test.c

medipix_sim_LDFLAGS += -lpthread

medipix_test_LIBS += medipix_low

# ------------------------
# Build the Area Detector Derived Library
# ------------------------

LIBRARY += medipixDetector
PROD += medipixDetectorApp

USR_INCLUDES += -I$(AREA_DETECTOR)/ADApp/cbfSrc

medipixDetector_SRCS += medipixDetector.cpp

# <name>.dbd will be created from <name>Include.dbd
DBD += medipixDetectorApp.dbd
#DBD += medipixDetectorSupport.dbd

# <name>_registerRecordDeviceDriver.cpp will be created from <name>.dbd
medipixDetectorApp_SRCS += medipixDetectorApp_registerRecordDeviceDriver.cpp medipixMain.cpp

# Add locally compiled object code
medipixDetectorApp_LIBS += medipixDetector
medipixDetectorApp_LIBS += cbfad

#include $(TOP)/ADApp/commonDriverMakefile
medipixDetector_LIBS    += ADBase netCDF
medipixDetectorApp_LIBS += NDPlugin ADBase netCDF
medipixDetectorApp_LIBS += calc busy sscan autosave mca
medipixDetector_LIBS    += calc busy sscan autosave
medipixDetector_LIBS    += asyn
medipixDetector_LIBS 	+= asyn
medipixDetector_LIBS    += $(EPICS_BASE_IOC_LIBS)
medipixDetectorApp_LIBS += $(EPICS_BASE_IOC_LIBS)

medipixDetector_SYS_LIBS			+= tiff jpeg z
medipixDetectorApp_SYS_LIBS  		+= tiff jpeg z

medipixDetector_SYS_LIBS      		+= gomp X11 xml2 png12 bz2 Xext freetype 
medipixDetectorApp_SYS_LIBS     	+= gomp X11 xml2 png12 bz2 Xext freetype 

include $(TOP)/configure/RULES
