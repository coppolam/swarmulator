# Thanks to the help from
# https://www.cs.swarthmore.edu/~newhall/unixhelp/howto_makefiles.html#creating
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings

APPNAME = swarmulator

CC = g++
CFLAGS += -g -Wall -std=gnu++0x -DDEBUG -DINFO

OPT=-lglut -lGLU -lGL -lXi -lXmu -lglfw -lpthread

SIMULATION_FOLDER = sw/simulation
ANIMATION_FOLDER = sw/animation
LOGGER_FOLDER = sw/logger
MATH_FOLDER = sw/math

INC=-I. -Isw -I$(SIMULATION_FOLDER) -I$(ANIMATION_FOLDER) -Isw/simulation/agents -Isw/simulation/controllers  -I$(LOGGER_FOLDER) -I$(MATH_FOLDER)

$(info ************ SWARMULATOR V0.1 **********)

# Build the executable
all: 
	$(CC) $(CFLAGS) $(INC) \
	-o \
	$(APPNAME) \
	sw/main.cpp \
	$(SIMULATION_FOLDER)/agents/particle.cpp \
	$(SIMULATION_FOLDER)/controllers/controller_bearing_shape.cpp \
	$(SIMULATION_FOLDER)/*.cpp \
	$(ANIMATION_FOLDER)/*.cpp \
	$(LOGGER_FOLDER)/*.cpp \
	$(MATH_FOLDER)/randomgenerator.c \
	$(OPT)

clean: 
	$(RM) $(APPNAME)
