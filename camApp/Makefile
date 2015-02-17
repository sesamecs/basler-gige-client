TOP=..

include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

PROD_HOST		+= cam
cam_SRCS		+= cam.c colormap.c
cam_INCLUDES	+= -I/usr/include/SDL -I/usr/local/include
cam_SYS_LIBS	+= SDL GL AntTweakBar
cam_LIBS		+= $(EPICS_BASE_HOST_LIBS)

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE
