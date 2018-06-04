# Thanks to the help from
# https://www.cs.swarthmore.edu/~newhall/unixhelp/howto_makefiles.html#creating
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings

TARGET = swarmulator

CC = g++
CFLAGS += -g -Wall -std=gnu++0x -DDEBUG -DINFO

OPT=-lglut -lGLU -lGL -lXi -lXmu -lglfw -lpthread -lxerces-c -Wno-deprecated-declarations

SIMULATION_FOLDER = sw/simulation
ANIMATION_FOLDER = sw/animation
LOGGER_FOLDER = sw/logger
MATH_FOLDER = sw/math

INC=-I. -Isw -I$(SIMULATION_FOLDER) -I$(ANIMATION_FOLDER) -Isw/simulation/agents -Isw/simulation/controllers  -I$(LOGGER_FOLDER) -I$(MATH_FOLDER) 

# Build the executable
all: 
	xsd cxx-tree --root-element-all conf/parameters.xsd
	@echo "Building $(TARGET)...";
	$(CC) $(CFLAGS) $(INC) -o $(TARGET) sw/main.cpp \
	parameters.cxx \
	$(SIMULATION_FOLDER)/agents/particle.cpp \
	$(SIMULATION_FOLDER)/controllers/*.cpp \
	$(SIMULATION_FOLDER)/*.cpp \
	$(ANIMATION_FOLDER)/*.cpp \
	$(LOGGER_FOLDER)/*.cpp \
	$(MATH_FOLDER)/randomgenerator.c \
	$(OPT);
	@echo "Done";

clean:
	@echo "Cleaning $(TARGET)...";
	$(RM) -r $(TARGET) parameters.*;
	@echo "Done";
