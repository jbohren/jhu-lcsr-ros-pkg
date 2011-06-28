################################################################################
# Unpacking CISST
#
# Makefile to build CISST Libraries
# author: Jonathan Bohren
# created: June 10, 2011
#
# Targets:
# 	all: cisst
# 	unpack: download and configure (but don't build) CISST
# 	configure: run cmake with the configuration given in this makefile
# 	cisst: unpack, configure, and build CISST, install to this ROS package
# 	doc: build CISST doxygen documentation
#		tidy: remove source and build tree (leaves libs, headers, etc)
#		clean: tidy and also removes all build producs
#
# NOTE: CISST has several external dependencies (FLTK, QT, ICE, etc). These
# dependencies are mentioned in manifest.xml, and are described by the
# rosdep.yaml dependency description file in the top level of the cisst_ros ROS
# stack.
#	
#	To check for these dependencies, call:
#	$> rosdep check cisst
#
# To install these depdendencies, simply call:
# $> rosdep install cisst
#
# For more info on rosdep, see http:///www.ros.org/wiki/rosdep
#
# More documentation below, if you're curious...
#
################################################################################

#include $(shell rospack find mk)/cmake.mk

all: cisst

################################################################################
# Unpacking CISST
#
# The ROS package "mk" includes scripts for pulling, patching, and building
# external libraries to be wrapped in ROS packages.
#
# For more info on how this is used, see http://www.ros.org/wiki/mk
################################################################################

# CISST frozen trunk Tarball
FILENAME = cisst_r2686_trunk.tar.gz
TARBALL = build/$(FILENAME)
TARBALL_URL = http://jhu-lcsr-ros-pkg.googlecode.com/files/$(FILENAME)
SOURCE_DIR = build/source
UNPACK_CMD = tar xzf
TARBALL_PATCH = patches/doc.patch

# Generate external library make targets
include $(shell rospack find mk)/download_unpack_build.mk

################################################################################
# Declare CISST build configuration variables (CMAKE flags) #
#
# Both ROS and CISST use CMake to build libraries, but these build systesma are
# completely independent. When building the CISST libraries, you can consider
# the ROS build system _delegating_ to the CISST build system, by calling
# "cmake" and "make" from within CISST's build libraries.
#
# For more info on the CISST build setup, see https://trac.lcsr.jhu.edu/cisst 
# For more info on these parameters, run ccmake as described on the CISST trac
################################################################################

# Get CISST ros package path (this is where we will install the CISST libs)
CISST_PACKAGE_PATH = $(shell rospack find cisst)

# CISST's CMake Arguments

# Generic build args
CISST_BUILD_ARGS = -D CMAKE_BUILD_TYPE="Release" \
									 -D CMAKE_INSTALL_PREFIX="$(CISST_PACKAGE_PATH)"

# Executables to build
CISST_BUILD_BINS_ARGS = -D BUILD_EXAMPLES=OFF \
											 -D BUILD_TESTS=OFF \
											 -D BUILD_UTILITIES=ON

# Different parts of the documentation
CISST_BUILD_DOC_ARGS = -D BUILD_DOC=OFF \
											 -D BUILD_DOC_doxygen=ON \
											 -D BUILD_DOC_release-notes=OFF

# Specify which CISST libraries are to be built
# Note that enabling some of these libraries necessitates external dependencies
# and configuration options (below)
CISST_BUILD_LIBS_ARGS = -D BUILD_LIBS_cisst3DUserInterfac=OFF \
												-D BUILD_LIBS_cisstCommon=ON \
												-D BUILD_LIBS_cisstDevices=ON \
												-D BUILD_LIBS_cisstInteractive=OFF \
												-D BUILD_LIBS_cisstMultiTask=ON \
												-D BUILD_LIBS_cisstNumerical=ON \
												-D BUILD_LIBS_cisstOSAbstraction=ON \
												-D BUILD_LIBS_cisstParameterTypes=ON \
												-D BUILD_LIBS_cisstRobot=ON \
												-D BUILD_LIBS_cisstStereoVision=ON\
												-D BUILD_LIBS_cisstVector=ON

# cisstNetLib is required for the cisstNumerical library
# These flags will instruct the CISST build system to automatically download and
# build the library.
CISSTNETLIB_ARGS = -D CISSTNETLIB_DOWNLOAD_NOW=ON \
									 -D CISSTNETLIB_DOWNLOAD_ARCHITECTURE="$(shell uname -m)"

# Flags for configuring realtime threading
CISST_REALTIME_ARGS = -D CISST_HAS_LINUX_RTAI=OFF \
											-D CISST_HAS_LINUX_XENOMAI=OFF

# QT configuration
CISST_QT_ARGS = -D CISST_HAS_QT=ON

# ICE is a CORBA-based middleware used in CISST for distributed communications
CISST_ICE_ARGS = -D CISST_MTS_HAS_ICE=ON

# SVL external libs
CISST_SVL_ARGS = -D CISST_SVL_DC1394_ENABLE_800Mbp=ON \
								 -D CISST_SVL_HAS_DC1394=ON \
								 -D CISST_SVL_HAS_FFMPEG=ON \
								 -D CISST_SVL_HAS_JPEG=ON \
								 -D CISST_SVL_HAS_MIL=OFF \
								 -D CISST_SVL_HAS_OPENCV=ON \
								 -D CISST_SVL_HAS_OPENCV2=ON \
								 -D CISST_SVL_HAS_PNG=ON \
								 -D CISST_SVL_HAS_VIDEO4LINUX2=ON \
								 -D CISST_SVL_HAS_X11=ON \
								 -D CISST_SVL_HAS_XV=OFF \
								 -D CISST_SVL_HAS_ZLIB=ON

# Get opencv from ROS! (this is an environment variable read by CISST)
# In the future it should probably be a CMAKE argument
export OpenCV_ROOT_DIR=$(shell rospack find opencv2)/opencv

# Hardware devices
CISST_DEV_ARGS = -D CISST_DEV_HAS_ACCELEGLOVE=OFF \
								 -D CISST_DEV_HAS_BITCTRL=OFF \
								 -D CISST_DEV_HAS_LOPOMOCO=OFF \
								 -D CISST_DEV_HAS_MICRONTRACKER=OFF \
								 -D CISST_DEV_HAS_NOVINTHDL=OFF \
								 -D CISST_DEV_HAS_OPENIGTLINK=OFF \
								 -D CISST_DEV_HAS_ROBOTCOMPONENTS=OFF \
								 -D CISST_DEV_HAS_RTSOCKETCAN=OFF \
								 -D CISST_DEV_HAS_SENSABLEHD=OFF

# Concatenate all the CMake arguments
CMAKE_ARGS = $(CISST_BUILD_ARGS) \
						 $(CISST_BUILD_BINS_ARGS) \
						 $(CISST_BUILD_DOC_ARGS) \
						 $(CISST_BUILD_LIBS_ARGS) \
						 $(CISSTNETLIB_ARGS) \
						 $(CISST_REALTIME_ARGS) \
						 $(CISST_QT_ARGS) \
						 $(CISST_ICE_ARGS) \
						 $(CISST_SVL_ARGS) \
						 $(CISST_DEV_ARGS)

# Args for just building documentation
# Turn off all libs, executables, etc
CMAKE_DOC_TARGET_ARGS = $(CISST_BUILD_ARGS) \
												-D BUILD_DOC=ON \
												-D BUILD_DOC_doxygen=ON \
												-D BUILD_DOC_release-notes=OFF

################################################################################
# Build Targets
################################################################################

# CISST Build directory
CISST_BUILD_DIR = $(SOURCE_DIR)/../build

# Target to just download and set up the build system
unpack: $(SOURCE_DIR)/unpacked
	mkdir -p $(CISST_BUILD_DIR)

# Apply the configuration as specified by $(CMAKE_ARGS), above
# NOTE: This needs to be done twice to incorporate CISSTNetlib
configure: unpack
	cd $(CISST_BUILD_DIR) && cmake $(CMAKE_ARGS) ../source
	cd $(CISST_BUILD_DIR) && cmake $(CMAKE_ARGS) ../source

# Build cisst and install it locally into the cisst ros package
# Additionally, place a flag that will keep rosbuild from trying to rebuild it
cisst: configure
	cd $(CISST_BUILD_DIR) && make && make install
	touch ROS_NOBUILD

# Build doxygen documentation
doc: unpack
	cd $(CISST_BUILD_DIR) && cmake $(CMAKE_DOC_TARGET_ARGS) ../source
	cd $(CISST_BUILD_DIR) && make && make install

# Remove everything but the installed libraries
tidy:
	rm -rf build 

# Remove all build files 
clean: tidy
	rm -rf bin include lib share ROS_NOBUILD
