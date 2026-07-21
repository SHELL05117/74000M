################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=
EXTRA_CFLAGS=
ROBOT_PROFILE?=492Z
VALID_ROBOT_PROFILES:=492X 492Z
ifeq ($(filter $(ROBOT_PROFILE),$(VALID_ROBOT_PROFILES)),)
$(error ROBOT_PROFILE must be 492X or 492Z)
endif
ROBOT_SOURCE_COMMIT:=$(shell git rev-parse HEAD 2>/dev/null || echo UNKNOWN)
ROBOT_BUILD_DIRTY:=$(shell test -z "$$(git status --porcelain --untracked-files=normal 2>/dev/null)" && echo 0 || echo 1)
EXTRA_CXXFLAGS=-DROBOT_SOURCE_COMMIT=\"$(ROBOT_SOURCE_COMMIT)\" -DROBOT_BUILD_DIRTY=$(ROBOT_BUILD_DIRTY) -DROBOT_PROFILE_$(ROBOT_PROFILE)=1
CXX_STANDARD:=gnu++17

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# Add libraries you do not wish to include in the cold image here
# EXCLUDE_COLD_LIBRARIES:= $(FWDIR)/your_library.a
EXCLUDE_COLD_LIBRARIES:= 

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS! 
# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk
