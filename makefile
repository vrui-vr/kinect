########################################################################
# Makefile for Kinect 3D Video Capture Project.
# Copyright (c) 2010-2025 Oliver Kreylos
#
# This file is part of the WhyTools Build Environment.
# 
# The WhyTools Build Environment is free software; you can redistribute
# it and/or modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2 of the
# License, or (at your option) any later version.
# 
# The WhyTools Build Environment is distributed in the hope that it will
# be useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
# 
# You should have received a copy of the GNU General Public License
# along with the WhyTools Build Environment; if not, write to the Free
# Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
# 02111-1307 USA
########################################################################

# Directory containing the Vrui build system. The directory below
# matches the default Vrui installation; if Vrui's installation
# directory was changed during Vrui's installation, the directory below
# must be adapted.
VRUI_MAKEDIR := /usr/local/share/Vrui-12.0/make

# Set the following variable to the type of facade projector to be built
# into the Kinect library. There are currently three types:
# 0 - CPU-based projector (Kinect::Projector). Creates vertex array and
#     triangle mesh from depth frame on CPU.
# 1 - Vertex shader-based projector (Kinect::Projector2). Updates static
#     template vertex array with depth frame in a vertex shader, and
#     creates triangle mesh from depth frame on CPU.
# 2 - Geometry shader-based projector (Kinect::ShaderProjector). Updates
#     static template vertex array and static template triangle mesh in
#     a geometry shader.
# Which facade projector version is better depends on system hardware,
# number of Kinect facades rendered simultaneously, rendering modes,
# etc. I.e., to optimize performance, typical-case testing is in order.
# The following code selects a reasonable choice based on whether the
# local OpenGL supports the GL_EXT_gpu_shader4 extension required by
# the Kinect::Projector2 vertex shader-based projector.
KINECT_PROJECTORTYPE=0
ifneq ($(strip $(shell glxinfo | grep GL_EXT_gpu_shader4)),)
  KINECT_PROJECTORTYPE = 1
endif

# Set configuration flags based on projector type choice:
KINECT_USE_PROJECTOR2 = 0
KINECT_USE_SHADERPROJECTOR = 0
ifeq ($(KINECT_PROJECTORTYPE),1)
  KINECT_USE_PROJECTOR2 = 1
endif
ifeq ($(KINECT_PROJECTORTYPE),2)
  KINECT_USE_SHADERPROJECTOR = 1
endif

########################################################################
# Everything below here should not have to be changed
########################################################################

PROJECT_NAME = Kinect
PROJECT_DISPLAYNAME = Kinect 3D Video Capture Project

# Specify version of created dynamic shared libraries
PROJECT_MAJOR = 4
PROJECT_MINOR = 3
PROJECT_NUMERICVERSION = 4003

# Root directory for Collaboration configuration data underneath Vrui's
# configuration directory:
KINECTETCDIREXT = $(PROJECT_FULLNAME)

# Root directory for Collaboration resource data underneath Vrui's
# shared data directory:
KINECTSHAREDIREXT = $(PROJECT_FULLNAME)

# Include definitions for the system environment and system-provided
# packages
include $(VRUI_MAKEDIR)/SystemDefinitions
include $(VRUI_MAKEDIR)/Packages.System
include $(VRUI_MAKEDIR)/Configuration.Vrui
include $(VRUI_MAKEDIR)/Packages.Vrui

# Check if Vrui's collaboration infrastructure is installed
-include $(VRUI_MAKEDIR)/Configuration.Collaboration
-include $(VRUI_MAKEDIR)/Packages.Collaboration
ifdef COLLABORATION_VERSION
  HAVE_COLLABORATION = 1
else
  HAVE_COLLABORATION = 0
endif

# Don't build the collaboration components until they've been ported to
# version 2 of the collaboration framework:
HAVE_COLLABORATION = 0

include $(PROJECT_ROOT)/BuildRoot/Packages.Kinect

# Set destination directories for libraries and plug-ins:
PROJECT_VISLETDIR = $(PROJECT_ROOT)/$(PLUGINDIR)/Vislets
PROJECT_PLUGINDIR = $(PROJECT_ROOT)/$(PLUGINDIR)/Plugins

# Set installation directories:
HEADERINSTALLDIR = $(VRUI_HEADERINSTALLDIR)/Kinect
LIBINSTALLDIR = $(VRUI_LIBINSTALLDIR)
EXECUTABLEINSTALLDIR = $(VRUI_EXECUTABLEINSTALLDIR)
VISLETINSTALLDIR = $(VRUI_PLUGININSTALLDIR)/$(VRVISLETSDIREXT)
ETCINSTALLDIR = $(VRUI_ETCINSTALLDIR)/$(KINECTETCDIREXT)
SHAREINSTALLDIR = $(VRUI_SHAREINSTALLDIR)/$(KINECTSHAREDIREXT)
SHADERINSTALLDIR = $(SHAREINSTALLDIR)/Shaders
MAKEINSTALLDIR = $(VRUI_MAKEINSTALLDIR)

# Set default destination directory for Kinect udev rule:
# FIXME: This should come from Configuration.Vrui!
ifeq ($(EXECUTABLEINSTALLDIR),/usr/bin)
  UDEVRULEDIR := /usr/lib/udev/rules.d
else
  UDEVRULEDIR := /etc/udev/rules.d
endif

########################################################################
# Specify additional compiler and linker flags
########################################################################

CFLAGS += -Wall -pedantic

# Override the include file and library search directories:
EXTRACINCLUDEFLAGS += -I$(PROJECT_ROOT)
EXTRALINKDIRFLAGS += -L$(PROJECT_LIBDIR)

########################################################################
# List packages used by this project
# (Supported packages can be found in
# $(VRUI_MAKEDIR)/BuildRoot/Packages)
########################################################################

PACKAGES = 

########################################################################
# Specify all final targets
########################################################################

LIBRARIES = 
PLUGINS = 
VISLETS = 
EXECUTABLES = 

#
# The Kinect driver library:
#

LIBRARY_NAMES = libKinect

LIBRARIES += $(LIBRARY_NAMES:%=$(call LIBRARYNAME,%))

#
# The Kinect executables:
#

EXECUTABLES += $(EXEDIR)/KinectUtil \
               $(EXEDIR)/RawKinectViewer \
               $(EXEDIR)/CalibrateCameras \
               $(EXEDIR)/KinectServer \
               $(EXEDIR)/KinectViewer
ifneq ($(KINECT_USE_PROJECTOR2),0)
  EXECUTABLES += $(EXEDIR)/BackgroundViewer
endif

#
# The Kinect vislets:
#

VISLET_NAMES = KinectViewer

VISLETNAME = $(PROJECT_VISLETDIR)/lib$(1).$(PLUGINFILEEXT)
VISLETS += $(VISLET_NAMES:%=$(call VISLETNAME,%))

#
# The Kinect collaboration infrastructure plug-ins:
#

COLLABORATIONPLUGIN_NAMES = KinectServer \
                            KinectClient

ifneq ($(HAVE_COLLABORATION),0)
  PLUGINS += $(COLLABORATIONPLUGIN_NAMES:%=$(call PLUGINNAME,%))
endif

# The make configuration file:
MAKECONFIGFILE = BuildRoot/Configuration.Kinect

ALL = $(LIBRARIES) $(EXECUTABLES) $(PLUGINS) $(VISLETS) $(MAKECONFIGFILE)

.PHONY: all
all: $(ALL)

# Make all components depend on Kinect driver library:
$(VISLETS) $(PLUGINS) $(EXECUTABLES): $(call LIBRARYNAME,libKinect)

########################################################################
# Pseudo-target to print configuration options and configure libraries
########################################################################

.PHONY: config config-invalidate
config: config-invalidate $(DEPDIR)/config

config-invalidate:
	@mkdir -p $(DEPDIR)
	@touch $(DEPDIR)/Configure-Begin

$(DEPDIR)/Configure-Begin:
	@mkdir -p $(DEPDIR)
ifeq ($(SYSTEM_HAVE_LIBUSB1),0)
	@echo "ERROR: Vrui was not built with libusb-1 support. Please install missing development package(s) and rebuild Vrui"
	@exit 1
endif
	@echo "---- $(PROJECT_FULLDISPLAYNAME) configuration options: ----"
ifneq ($(SYSTEM_HAVE_LIBJPEG),0)
	@echo "Support for Microsoft Kinect v2 (Kinect-for-Xbox-One) enabled"
else
	@echo "Support for Microsoft Kinect v2 (Kinect-for-Xbox-One) disabled due to missing jpeg library"
endif
ifneq ($(SYSTEM_HAVE_REALSENSE),0)
	@echo "Support for Intel RealSense cameras via first-generation librealsense library enabled"
else
	@echo "Support for Intel RealSense cameras via first-generation librealsense library disabled"
endif
ifneq ($(KINECT_USE_PROJECTOR2),0)
	@echo "GLSL vertex shader-based facade projector selected"
else
ifneq ($(KINECT_USE_SHADERPROJECTOR),0)
	@echo "GLSL geometry shader-based facade projector selected"
else
	@echo "CPU-based facade projector selected"
endif
endif
ifneq ($(HAVE_COLLABORATION),0)
	@echo "Vrui collaboration infrastructure detected"
endif
	@touch $(DEPDIR)/Configure-Begin

$(DEPDIR)/Configure-Kinect: $(DEPDIR)/Configure-Begin
	@cp Kinect/Config.h.template Kinect/Config.h.temp
	@$(call CONFIG_SETVAR,Kinect/Config.h.temp,KINECT_CONFIG_HAVE_KINECTV2,$(SYSTEM_HAVE_LIBJPEG))
	@$(call CONFIG_SETVAR,Kinect/Config.h.temp,KINECT_CONFIG_HAVE_LIBREALSENSE,$(SYSTEM_HAVE_REALSENSE))
	@$(call CONFIG_SETVAR,Kinect/Config.h.temp,KINECT_CONFIG_USE_PROJECTOR2,$(KINECT_USE_PROJECTOR2))
	@$(call CONFIG_SETVAR,Kinect/Config.h.temp,KINECT_CONFIG_USE_SHADERPROJECTOR,$(KINECT_USE_SHADERPROJECTOR))
	@if ! diff -qN Kinect/Config.h.temp Kinect/Config.h > /dev/null ; then cp Kinect/Config.h.temp Kinect/Config.h ; fi
	@rm Kinect/Config.h.temp
	@cp Kinect/Internal/Config.h.template Kinect/Internal/Config.h.temp
	@$(call CONFIG_SETSTRINGVAR,Kinect/Internal/Config.h.temp,KINECT_INTERNAL_CONFIG_CONFIGDIR,$(ETCINSTALLDIR))
	@$(call CONFIG_SETSTRINGVAR,Kinect/Internal/Config.h.temp,KINECT_INTERNAL_CONFIG_SHADERDIR,$(SHADERINSTALLDIR))
	@if ! diff -qN Kinect/Internal/Config.h.temp Kinect/Internal/Config.h > /dev/null ; then cp Kinect/Internal/Config.h.temp Kinect/Internal/Config.h ; fi
	@rm Kinect/Internal/Config.h.temp
	@touch $(DEPDIR)/Configure-Kinect

$(DEPDIR)/Configure-Install: $(DEPDIR)/Configure-Kinect
	@echo "---- $(PROJECT_FULLDISPLAYNAME) installation configuration ----"
	@echo "Root installation directory: $(VRUI_PACKAGEROOT)"
	@echo "Library directory: $(LIBINSTALLDIR)"
	@echo "Executable directory: $(EXECUTABLEINSTALLDIR)"
	@echo "Vislet plug-in directory: $(VISLETINSTALLDIR)"
ifneq ($(HAVE_COLLABORATION),0)
	@echo "Collaboration protocol plug-in directory: $(COLLABORATIONPLUGININSTALLDIR)"
endif
	@echo "Calibration data directory: $(ETCINSTALLDIR)"
	@echo "Resource data directory: $(SHAREINSTALLDIR)"
	@echo "Shader directory: $(SHADERINSTALLDIR)"
	@echo "Makefile directory: $(MAKEINSTALLDIR)"
	@touch $(DEPDIR)/Configure-Install

$(DEPDIR)/Configure-End: $(DEPDIR)/Configure-Install
	@echo "---- End of $(PROJECT_FULLDISPLAYNAME) configuration options ----"
	@touch $(DEPDIR)/Configure-End

$(DEPDIR)/config: $(DEPDIR)/Configure-End
	@touch $(DEPDIR)/config

########################################################################
# Specify other actions to be performed on a `make clean'
########################################################################

.PHONY: extraclean
extraclean:
	-rm -f $(LIBRARY_NAMES:%=$(PROJECT_LIBDIR)/$(call DSONAME,%))
	-rm -f $(LIBRARY_NAMES:%=$(PROJECT_LIBDIR)/$(call LINKDSONAME,%))

.PHONY: extrasqueakyclean
extrasqueakyclean:
	-rm -f $(ALL)

# Include basic makefile
include $(VRUI_MAKEDIR)/BasicMakefile

########################################################################
# Specify build rules for dynamic shared objects
########################################################################

# Set up a link rpath to find package-created libraries and plug-ins
# while building dependent objects:
EXTRALINKDIRFLAGS = -Wl,-rpath-link=$(PROJECT_LIBDIR) -L$(PROJECT_LIBDIR)

# Headers and sources to support Kinect v2:
LIBKINECT_KINECTV2_SOURCES = Kinect/CameraV2.cpp \
                             Kinect/Internal/KinectV2CommandDispatcher.cpp \
                             Kinect/Internal/KinectV2JpegStreamReader.cpp \
                             Kinect/Internal/KinectV2DepthStreamReader.cpp

# Headers and sources to support RealSense cameras:
LIBKINECT_REALSENSE_SOURCES = Kinect/CameraRealSense.cpp \
                              Kinect/Internal/LibRealSenseContext.cpp

# Headers and sources for dummy implementations of certain 3D camera types:
LIBKINECT_DUMMY_SOURCES = Kinect/CameraV2Dummy.cpp \
                          Kinect/CameraRealSenseDummy.cpp

# Create list of libKinect external headers and sources:
LIBKINECT_HEADERS = $(wildcard Kinect/*.h)
LIBKINECT_SOURCES = $(filter-out $(LIBKINECT_KINECTV2_SOURCES) $(LIBKINECT_REALSENSE_SOURCES) $(LIBKINECT_DUMMY_SOURCES),$(wildcard Kinect/*.cpp) $(wildcard Kinect/Internal/*.cpp))

LIBKINECT_PACKAGES = MYVIDEO MYGLMOTIF MYIMAGES MYGLGEOMETRY MYGLSUPPORT MYGLWRAPPERS MYGEOMETRY MYMATH MYCLUSTER MYCOMM MYIO MYUSB MYREALTIME MYMISC GL LIBUSB1

# Add Kinect v2 sources if jpeg library is present, otherwise use dummy implementation:
ifneq ($(SYSTEM_HAVE_LIBJPEG),0)
  LIBKINECT_SOURCES += $(LIBKINECT_KINECTV2_SOURCES)
  LIBKINECT_PACKAGES += JPEG
else
  LIBKINECT_SOURCES += Kinect/CameraV2Dummy.cpp
endif

# Add RealSense sources if RealSense library is present, otherwise use dummy implementation:
ifneq ($(SYSTEM_HAVE_REALSENSE),0)
  LIBKINECT_SOURCES += $(LIBKINECT_REALSENSE_SOURCES)
  LIBKINECT_PACKAGES += REALSENSE
else
  LIBKINECT_SOURCES += Kinect/CameraRealSenseDummy.cpp
endif

# Make the Kinect library depend on configuration:
$(call LIBOBJNAMES,$(LIBKINECT_SOURCES)): | $(DEPDIR)/config

$(call LIBRARYNAME,libKinect): PACKAGES = $(LIBKINECT_PACKAGES)
$(call LIBRARYNAME,libKinect): $(call LIBOBJNAMES,$(LIBKINECT_SOURCES))
.PHONY: libKinect
libKinect: $(call LIBRARYNAME,libKinect)

# Everything except libKinect depends on libKinect:
$(filter-out $(call LIBRARYNAME,libKinect),$(ALL)): | $(call LIBRARYNAME,libKinect)

########################################################################
# Specify build rules for executables
########################################################################

# Make everything depend on configuration:
OBJNAMES = $(1:%.cpp=$(OBJDIR)/%.o)
$(call OBJNAMES,$(wildcard *.cpp)): | $(DEPDIR)/config
$(call PLUGINOBJNAMES,$(wildcard *.cpp)): | $(DEPDIR)/config

#
# Utility to list Kinect devices on all USB buses and send commands to
# them:
#

$(EXEDIR)/KinectUtil: PACKAGES += MYKINECT MYMATH MYIO MYUSB MYMISC
$(EXEDIR)/KinectUtil: $(OBJDIR)/KinectUtil.o
.PHONY: KinectUtil
KinectUtil: $(EXEDIR)/KinectUtil

#
# Viewer for raw depth and color image streams, with ability to
# internally and externally calibrate Kinect cameras:
#

$(EXEDIR)/RawKinectViewer: PACKAGES += MYKINECT MYVRUI MYGLMOTIF MYIMAGES MYGLSUPPORT MYGLWRAPPERS MYGEOMETRY MYMATH MYIO MYTHREADS MYMISC GL
$(EXEDIR)/RawKinectViewer: $(OBJDIR)/PauseTool.o \
                           $(OBJDIR)/MeasurementTool.o \
                           $(OBJDIR)/TiePointTool.o \
                           $(OBJDIR)/LineTool.o \
                           $(OBJDIR)/DepthCorrectionTool.o \
                           $(OBJDIR)/GridTool.o \
                           $(OBJDIR)/PlaneTool.o \
                           $(OBJDIR)/PointPlaneTool.o \
                           $(OBJDIR)/CalibrationCheckTool.o \
                           $(OBJDIR)/RawKinectViewer.o
.PHONY: RawKinectViewer
RawKinectViewer: $(EXEDIR)/RawKinectViewer

#
# Utility to calculate an extrinsic calibration transformation between a
# 3D camera and a 6-DOF tracking system, using a tracked controller and
# a disk target.
#

$(EXEDIR)/ExtrinsicCalibrator: PACKAGES += MYKINECT MYVRUI MYGLMOTIF MYGLGEOMETRY MYGEOMETRY MYMATH MYCOMM MYIO MYTHREADS MYMISC GL
$(EXEDIR)/ExtrinsicCalibrator: $(OBJDIR)/ExtrinsicCalibrator.o
.PHONY: ExtrinsicCalibrator
ExtrinsicCalibrator: $(EXEDIR)/ExtrinsicCalibrator

#
# Utility to view and edit captured background removal frames.
#

$(EXEDIR)/BackgroundViewer: PACKAGES += MYKINECT MYVRUI MYGLGEOMETRY MYGLSUPPORT MYGEOMETRY MYMATH MYIO MYTHREADS MYMISC GL
$(EXEDIR)/BackgroundViewer: $(OBJDIR)/BackgroundViewer.o
.PHONY: BackgroundViewer
BackgroundViewer: $(EXEDIR)/BackgroundViewer

#
# Server for real-time 3D video streaming protocol:
#

# Tell Kinect server to print status info:
$(OBJDIR)/KinectServer.o: CFLAGS += -DVERBOSE

# Tell Kinect server to be extremely verbose:
#$(OBJDIR)/KinectServer.o: CFLAGS += -DVVERBOSE

$(EXEDIR)/KinectServer: PACKAGES += MYKINECT MYVIDEO MYGEOMETRY MYCOMM MYIO MYUSB MYTHREADS MYMISC
$(EXEDIR)/KinectServer: $(OBJDIR)/KinectServer.o \
                        $(OBJDIR)/KinectServerMain.o
.PHONY: KinectServer
KinectServer: $(EXEDIR)/KinectServer

#
# Viewer for 3D image streams from one or more Kinect devices, pre-
# recorded files or 3D video streaming servers:
#

$(EXEDIR)/KinectViewer: PACKAGES += MYKINECT MYVRUI MYSCENEGRAPH MYSOUND MYGLMOTIF MYIMAGES MYGLGEOMETRY MYGLSUPPORT MYGLWRAPPERS MYGEOMETRY MYMATH MYCOMM MYIO MYTHREADS MYMISC GL
$(EXEDIR)/KinectViewer: $(OBJDIR)/SphereExtractor.o \
                        $(OBJDIR)/SphereExtractorTool.o \
                        $(OBJDIR)/KinectViewer.o
.PHONY: KinectViewer
KinectViewer: $(EXEDIR)/KinectViewer

#
# Several obsolete or testing utilities or applications:
#

$(EXEDIR)/CompressDepthFile: PACKAGES += MYKINECT MYIO MYMISC
$(EXEDIR)/CompressDepthFile: $(OBJDIR)/CompressDepthFile.o
.PHONY: CompressDepthFile
CompressDepthFile: $(EXEDIR)/CompressDepthFile

$(EXEDIR)/DepthCompressionTest: PACKAGES += MYKINECT MYIO MYMISC
$(EXEDIR)/DepthCompressionTest: $(OBJDIR)/DepthCompressionTest.o
.PHONY: DepthCompressionTest
DepthCompressionTest: $(EXEDIR)/DepthCompressionTest

$(EXEDIR)/ColorCompressionTest: PACKAGES += MYKINECT MYIO MYMISC
$(EXEDIR)/ColorCompressionTest: $(OBJDIR)/ColorCompressionTest.o
.PHONY: ColorCompressionTest
ColorCompressionTest: $(EXEDIR)/ColorCompressionTest

$(EXEDIR)/CalibrateDepth: PACKAGES += MYKINECT MYGEOMETRY MYMATH MYIO MYMISC
$(EXEDIR)/CalibrateDepth: $(OBJDIR)/CalibrateDepth.o
.PHONY: CalibrateDepth
CalibrateDepth: $(EXEDIR)/CalibrateDepth

$(EXEDIR)/CalibrateCameras: PACKAGES += MYMATH MYIO
$(EXEDIR)/CalibrateCameras: $(OBJDIR)/CalibrateCameras.o
.PHONY: CalibrateCameras
CalibrateCameras: $(EXEDIR)/CalibrateCameras

$(EXEDIR)/NewCalibrateCameras: PACKAGES += MYGEOMETRY MYMATH MYIO
$(EXEDIR)/NewCalibrateCameras: $(OBJDIR)/NewCalibrateCameras.o
.PHONY: NewCalibrateCameras
NewCalibrateCameras: $(EXEDIR)/NewCalibrateCameras

$(EXEDIR)/TestAlignment: PACKAGES += MYGEOMETRY MYMATH MYIO
$(EXEDIR)/TestAlignment: $(OBJDIR)/TestAlignment.o
.PHONY: TestAlignment
TestAlignment: $(EXEDIR)/TestAlignment

$(EXEDIR)/SpaceCarver: PACKAGES += MYKINECT MYGEOMETRY MYIO MYMISC
$(EXEDIR)/SpaceCarver: $(OBJDIR)/SpaceCarver.o
.PHONY: SpaceCarver
SpaceCarver: $(EXEDIR)/SpaceCarver

########################################################################
# Specify build rules for vislet plug-ins
########################################################################

# Make everything depend on configuration:
$(call PLUGINOBJNAMES,$(wildcard *.cpp)): | $(DEPDIR)/config

# Implicit rule to link vislet plug-ins:
$(call VISLETNAME,%): PACKAGES = MYKINECT MYVRUI
$(call VISLETNAME,%): $(call PLUGINOBJNAMES,Vislets/%.cpp)
	@mkdir -p $(PROJECT_VISLETDIR)
ifdef SHOWCOMMAND
	$(CCOMP) $(PLUGINLINKFLAGS) -o $@ $^ $(PLUGINLFLAGS)
else
	@echo Linking $@...
	@$(CCOMP) $(PLUGINLINKFLAGS) -o $@ $^ $(PLUGINLFLAGS)
endif

#
# Vislet to render live 3D video in any Vrui application:
#

$(call VISLETNAME,KinectViewer): PACKAGES += MYVIDEO MYGLGEOMETRY MYGEOMETRY MYMATH MYCOMM MYIO MYTHREADS MYMISC GL

#
# Vislet to record 3D video from any Vrui application:
#

$(call VISLETNAME,KinectRecorder): $(OBJDIR)/pic/Vislets/KinectRecorder.o

#
# Vislet to play back previously recorded 3D video in any Vrui
# application:
#

$(call VISLETNAME,KinectPlayer): $(OBJDIR)/pic/Vislets/KinectPlayer.o

# Mark all vislet plugin object files as intermediate:
.SECONDARY: $(VISLET_NAMES:%=$(call PLUGINOBJNAMES,Vislets/%.cpp))

########################################################################
# Specify build rules for collaboration infrastructure plug-ins
########################################################################

# Implicit rule for creating collaboration infrastructure plug-ins:
$(call PLUGINNAME,%): $(call PLUGINOBJNAMES,Plugins/%.cpp)
	@mkdir -p $(PROJECT_PLUGINDIR)
ifdef SHOWCOMMAND
	$(CCOMP) $(PLUGINLINKFLAGS) -o $@ $^ $(PLUGINLFLAGS)
else
	@echo Linking $@...
	@$(CCOMP) $(PLUGINLINKFLAGS) -o $@ $^ $(PLUGINLFLAGS)
endif

#
# Server-side plugin:
#

$(call PLUGINNAME,KinectServer): PACKAGES += MYCOLLABORATIONSERVER
$(call PLUGINNAME,KinectServer): $(OBJDIR)/pic/Plugins/KinectProtocol.o \
                                 $(OBJDIR)/pic/Plugins/KinectServer.o

#
# Client-side plugin:
#

$(call PLUGINNAME,KinectClient): PACKAGES += MYKINECT MYCOLLABORATIONCLIENT
$(call PLUGINNAME,KinectClient): $(OBJDIR)/pic/Plugins/KinectProtocol.o \
                                 $(OBJDIR)/pic/Plugins/KinectClient.o

########################################################################
# Specify installation rules for header files, libraries, executables,
# configuration files, and shared files.
########################################################################

# Pseudo-target to dump Kinect configuration settings
$(MAKECONFIGFILE): config
	@echo Creating configuration makefile fragment...
	@echo '# Makefile fragment for $(PROJECT_DISPLAYNAME) configuration options' > $(MAKECONFIGFILE)
	@echo '# Autogenerated by $(PROJECT_DISPLAYNAME) installation on $(shell date)' >> $(MAKECONFIGFILE)
	@echo >> $(MAKECONFIGFILE)
	@echo '# Version information:'>> $(MAKECONFIGFILE)
	@echo 'KINECT_VERSION = $(PROJECT_NUMERICVERSION)' >> $(MAKECONFIGFILE)
	@echo 'KINECT_NAME = $(PROJECT_FULLNAME)' >> $(MAKECONFIGFILE)
	@echo >> $(MAKECONFIGFILE)
	@echo '# Configuration settings:'>> $(MAKECONFIGFILE)
	@echo 'KINECT_CONFIGDIR = $(ETCINSTALLDIR)' >> $(MAKECONFIGFILE)
	@echo 'KINECT_HAVE_COLLABORATION = $(HAVE_COLLABORATION)' >> $(MAKECONFIGFILE)

ifdef INSTALLPREFIX
  HEADERINSTALLDIR := $(INSTALLPREFIX)/$(HEADERINSTALLDIR)
  LIBINSTALLDIR := $(INSTALLPREFIX)/$(LIBINSTALLDIR)
  PLUGININSTALLDIR := $(INSTALLPREFIX)/$(PLUGININSTALLDIR)
  EXECUTABLEINSTALLDIR := $(INSTALLPREFIX)/$(EXECUTABLEINSTALLDIR)
  ETCINSTALLDIR := $(INSTALLPREFIX)/$(ETCINSTALLDIR)
  SHAREINSTALLDIR := $(INSTALLPREFIX)/$(SHAREINSTALLDIR)
  MAKEINSTALLDIR := $(INSTALLPREFIX)/$(MAKEINSTALLDIR)
endif

install:
# Install all header files in HEADERINSTALLDIR:
	@echo Installing header files...
	@install -d $(HEADERINSTALLDIR)
	@install -m u=rw,go=r $(LIBKINECT_HEADERS) $(HEADERINSTALLDIR)
# Install all library files in LIBINSTALLDIR:
	@echo Installing libraries...
	@install -d $(LIBINSTALLDIR)
	@install $(LIBRARIES) $(LIBINSTALLDIR)
	@echo Configuring run-time linker...
	$(foreach LIBNAME,$(LIBRARY_NAMES),$(call CREATE_SYMLINK,$(LIBNAME)))
# Install all binaries in EXECUTABLEINSTALLDIR:
	@echo Installing executables...
	@install -d $(EXECUTABLEINSTALLDIR)
	@install $(EXECUTABLES) $(EXECUTABLEINSTALLDIR)
# Install all vislet plugins in VISLETINSTALLDIR:
	@echo Installing vislet plugins...
	@install -d $(VISLETINSTALLDIR)
	@install $(VISLETS) $(VISLETINSTALLDIR)
ifneq ($(HAVE_COLLABORATION),0)
  # Install all collaboration protocol plugins in COLLABORATIONPLUGININSTALLDIR:
	@echo Installing collaboration protocol plugins...
	@install -d $(COLLABORATIONPLUGININSTALLDIR)
	@install $(PLUGINS) $(COLLABORATIONPLUGININSTALLDIR)
endif
# Install all configuration files in ETCINSTALLDIR:
	@echo Installing configuration files...
	@install -d $(ETCINSTALLDIR)
	@install -m u=rw,go=r $(PROJECT_ETCDIR)/KinectServer.cfg $(ETCINSTALLDIR)
# Install all resource files in SHAREINSTALLDIR:
	@echo Installing resource files...
	@install -d $(SHAREINSTALLDIR)
	@install -m u=rw,go=r $(PROJECT_SHAREDIR)/CalibrationTarget.svg $(SHAREINSTALLDIR)
	@install -d $(SHADERINSTALLDIR)
	@install -m u=rw,go=r $(PROJECT_SHAREDIR)/Shaders/* $(SHADERINSTALLDIR)
# Install the package and configuration files in MAKEINSTALLDIR:
	@echo Installing makefile fragments...
	@install -d $(MAKEINSTALLDIR)
	@install -m u=rw,go=r BuildRoot/Packages.Kinect $(MAKEINSTALLDIR)
	@install -m u=rw,go=r $(MAKECONFIGFILE) $(MAKEINSTALLDIR)

installudevrules:
# Install the Kinect udev rule in the udev rule directory:
	@echo Installing Kinect device permission udev rule in $(UDEVRULEDIR)
	@install -m u=rw,go=r share/69-Kinect.rules $(UDEVRULEDIR)

uninstall:
	@echo Removing header files...
	@rm -rf $(HEADERINSTALLDIR)
	@echo Removing libraries...
	@rm -f $(LIBRARIES:$(LIBDESTDIR)/%=$(LIBINSTALLDIR)/%)
	$(foreach LIBNAME,$(LIBRARY_NAMES),$(call DESTROY_SYMLINK,$(LIBNAME)))
	@echo Removing executables...
	@rm -f $(EXECUTABLES:$(EXEDIR)/%=$(EXECUTABLEINSTALLDIR)/%)
	@echo Removing vislet plugins...
	@rm -f $(VISLETS:$(VISLETDESTDIR)/%=$(VISLETINSTALLDIR)/%)
ifneq ($(HAVE_COLLABORATION),0)
	@echo Removing collaboration protocol plugins...
	@rm -f $(PLUGINS:$(PLUGINDESTDIR)/%=$(COLLABORATIONPLUGININSTALLDIR)/%)
endif
	@echo Removing configuration files...
	@rm -rf $(ETCINSTALLDIR)
	@echo Removing resource files...
	@rm -rf $(SHAREINSTALLDIR)
	@echo Removing makefile fragments...
	@rm -f $(MAKEINSTALLDIR)/Packages.Kinect
	@rm -f $(MAKEINSTALLDIR)/Configuration.Kinect

uninstalludevrules:
# Remove the Kinect udev rule from the udev rule directory:
	@echo Removing Kinect device permission udev rule from $(UDEVRULEDIR)
	@rm -f $(UDEVRULEDIR)/69-Kinect.rules

#
# Target to install a "live" version of the Kinect 3D Video Capture
# Project in the chosen installation directory that automatically
# reflects changes made to the Vrui Collaboration Infrastructure
# sources.
#

devinstall:
# Install all header files in HEADERINSTALLDIR:
	@echo Installing header files...
	@mkdir -p $(HEADERINSTALLDIR)
	@ln -sf $(LIBKINECT_HEADERS:%=$(PROJECT_ROOT)/%) $(HEADERINSTALLDIR)
# Install all library files in LIBINSTALLDIR:
	@echo Installing libraries...
	@mkdir -p $(LIBINSTALLDIR)
	@ln -sf $(LIBRARIES) $(LIBINSTALLDIR)
	@echo Configuring run-time linker...
	$(foreach LIBNAME,$(LIBRARY_NAMES),$(call CREATE_SYMLINK,$(LIBNAME)))
# Install all binaries in EXECUTABLEINSTALLDIR:
	@echo Installing executables...
	@mkdir -p $(EXECUTABLEINSTALLDIR)
	@ln -sf $(EXECUTABLES:%=$(PROJECT_ROOT)/%) $(EXECUTABLEINSTALLDIR)
# Install all vislet plug-ins in VISLETINSTALLDIR:
	@echo Installing vislet plugins...
	@mkdir -p $(VISLETINSTALLDIR)
	@ln -sf $(VISLETS) $(VISLETINSTALLDIR)
# Install all configuration files in ETCINSTALLDIR:
	@echo Installing configuration files...
	@mkdir -p $(ETCINSTALLDIR)
	@ln -sf $(PROJECT_ROOT)/$(PROJECT_ETCDIR)/* $(ETCINSTALLDIR)
# Install all resource files in SHAREINSTALLDIR:
	@echo Installing resource files...
	@mkdir -p $(SHAREINSTALLDIR)
	@ln -sf $(PROJECT_ROOT)/$(PROJECT_SHAREDIR)/CalibrationTarget.svg $(SHAREINSTALLDIR)
	@mkdir -p $(SHADERINSTALLDIR)
	@ln -sf $(PROJECT_ROOT)/$(PROJECT_SHAREDIR)/Shaders/* $(SHADERINSTALLDIR)
# Install the package and configuration files in MAKEINSTALLDIR:
	@echo Installing makefile fragments...
	@mkdir -p $(MAKEINSTALLDIR)
	@ln -sf $(PROJECT_ROOT)/BuildRoot/Packages.Kinect $(MAKEINSTALLDIR)
	@ln -sf $(PROJECT_ROOT)/$(MAKECONFIGFILE) $(MAKEINSTALLDIR)/Configuration.Kinect
