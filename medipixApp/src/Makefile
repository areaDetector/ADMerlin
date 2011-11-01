TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY_IOC += medipix_low

# The following are compiled and added to the support library
medipix_low_SRCS += medipix_low.c


# ------------------------
# Build an IOC Application
# ------------------------

PROD = medipix_sim
PROD += medipix_test

# Add locally compiled object code
medipix_sim_SRCS += medipix_sim.c
medipix_test_SRCS += medipix_test.c

medipix_test_LIBS += medipix_low



include $(TOP)/configure/RULES
