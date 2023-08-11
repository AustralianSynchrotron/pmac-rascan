TOP=../..

include $(TOP)/configure/CONFIG

# -------------------------------
# Build an Diamond Support Module
# -------------------------------

LIBRARY_IOC += pmacRascan

# xxxRecord.h will be created from xxxRecord.dbd
#DBDINC += xxx.h
#DBD += pmacRascanSupport.dbd

# xxxRecord.dbd will be installed into <top>/dbd
DBD += pmacRascan.dbd
pmacRascan_DBD += base.dbd
pmacRascan_DBD += asyn.dbd
pmacRascan_DBD += calcSupport.dbd
pmacRascan_DBD += stream.dbd
#pmacRascan_DBD += sscanSupport.dbd
pmacRascan_DBD += motorSupport.dbd
pmacRascan_DBD += busySupport.dbd
pmacRascan_DBD += stdSupport.dbd

# devExecute_Shell.dbd
pmacRascan_DBD += devExecute_Shell.dbd

# The following are compiled and added to the support library
#pmacUtil_SRCS += gather.st
#pmacUtil_SRCS += compensationtable.c
#pmacUtil_SRCS += pmacStatusOutParse.c

#pmacUtil_LIBS += seq pv

pmacRascan_LIBS += devExecute_Shell

# We need to link against the EPICS Base libraries
pmacRascan_LIBS += $(EPICS_BASE_IOC_LIBS)

include $(TOP)/configure/RULES
