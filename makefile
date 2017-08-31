# Thanks to the help from
# https://www.cs.swarthmore.edu/~newhall/unixhelp/howto_makefiles.html#creating
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings

APPNAME = swarmulator

CC = g++
CFLAGS += -g -Wall -std=gnu++0x -DDEBUG -DINFO

INC=-I. -Isw -Isw/animation/ -Isw/simulation -Isw/logger -Isw/math
OPT=-lglut -lGLU -lGL -lXi -lXmu -lglfw

$(info ************ SWARMULATOR V0.1 **********)

# Build the executable
all: 
	$(CC) $(CFLAGS) $(INC) \
	-o \
	$(APPNAME) \
	sw/main.cpp \
	sw/animation/draw.cpp \
	sw/simulation/particle.cpp \
	sw/simulation/agent.cpp \
	sw/simulation/controller.cpp \
	sw/simulation/omniscient_observer.cpp \
	sw/logger/txtwrite.cpp \
	sw/math/randomgenerator.c \
	$(OPT)

clean: 
	$(RM) $(APPNAME)
