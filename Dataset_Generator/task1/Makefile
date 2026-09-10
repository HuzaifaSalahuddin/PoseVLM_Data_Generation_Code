BASE = ../../../botop/rai
BASE2 = ../../../botop/src
BASE3 = ../../src

SRCS = main.cpp \
       $(BASE)/src/Control/TimingMPC.cpp \
       $(BASE)/src/Control/WaypointMPC.cpp \
       $(BASE)/src/Control/ShortPathMPC.cpp \
       $(BASE)/src/KOMO/manipTools.cpp \
       $(BASE)/src/Algo/SplineCtrlFeed.cpp \
       $(BASE)/src/Control/timingOpt.cpp \
       $(BASE3)/DL.cpp

OBJS = $(SRCS:.cpp=.o)

# Include directories
CXXFLAGS += -I$(BASE) -I$(BASE2) -I$(BASE3) -I$(BASE)/src/Control -I/usr/include/opencv4 -I/usr/include
CXXFLAGS += -I/home/huzaifa/anaconda3/envs/RAI/include/python3.8
CXXFLAGS += -I/home/huzaifa/anaconda3/envs/RAI/include

# Python-specific flags
CXXFLAGS += $(shell python3-config --includes)

# Library paths and linking flags
LDFLAGS += -L/home/huzaifa/anaconda3/envs/RAI/lib
LDFLAGS += $(shell python3-config --ldflags)
LDFLAGS += $(shell pkg-config --libs opencv4) -pthread

# Dependencies
DEPEND = Core Algo Gui Geo Kin Optim KOMO Franka Control

# Include generic makefile
include $(BASE)/_make/generic.mk

# Compilation rule
.cpp.o:
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Final build target
all: $(OBJS)
	$(CXX) $(CXXFLAGS) -o my_program $(OBJS) $(LDFLAGS)

