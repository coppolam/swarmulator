# Makefile for Swarmulator
#
# Mario Coppola, 2017-2018

TARGET = swarmulator # application name
BUILD_FOLDER = build
SRC_FOLDER = sw
# Use these to give input parameters
# IP1?=
# IP2?=
# IAW?=
# Compiler parameters
# Thanks to the help from
# https://www.cs.swarthmore.edu/~newhall/unixhelp/howto_makefiles.html#creating
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings
CC = g++ # chosen compiler
CFLAGS += -g -Wall -std=gnu++0x -DDEBUG -DINFO -D_GLIBCXX_USE_NANOSLEEP -DSWARMULATOR #-DP1=$(IP1) -DP2=$(IP2) -DARENAWALLS=$(IAW)
OPT=-lglut -lGLU -lGL -lpthread -lxerces-c -Wno-deprecated-declarations -fno-inline-functions

# General parameters to include all cpp files and all subfolders
INC_DIRS = $(shell find sw -maxdepth 50 -type d) # Max depth 50 layers. Should be enough.
INC_PARAMS = $(foreach d, $(INC_DIRS), -I$d) #Each include folder must have a -I before it
INC = -I. -I$(SRC_FOLDER) -I$(BUILD_FOLDER) $(INC_PARAMS) # All include paths
SOURCES_CPP = $(shell find $(SRC_FOLDER) -name *.cpp) # Recursively find all cpp 
SOURCES_CPP := $(filter-out sw/math/NEAT_v1.2.1/sw/%.cpp, $(SOURCES_CPP)) # Exclude NEAT if not relevant
SOURCES_C = $(shell find $(SRC_FOLDER) -name *.c) # Recursively find all cpp 
MAKE = $(CC) $(CFLAGS) $(INC)
OBJECTS_CPP=$(SOURCES_CPP:%.cpp=$(BUILD_FOLDER)/%.o)
OBJECTS_C=$(SOURCES_C:%.c=$(BUILD_FOLDER)/%.o)

# Build the executable
# Using @...; suppresses the output of the arguments
all: $(TARGET)

$(TARGET): xsd $(OBJECTS_C) $(OBJECTS_CPP)
	# Building $(TARGET)
	@$(MAKE) $(BUILD_FOLDER)/*.cxx  $(OBJECTS_C) $(OBJECTS_CPP) -o $@ $(OPT);

xsd:
	# Generating parameters file
	@mkdir -p $(BUILD_FOLDER);
	@xsdcxx cxx-tree --output-dir "$(BUILD_FOLDER)" --root-element-all conf/parameters.xsd;

$(BUILD_FOLDER)/%.o: %.cpp # This rule defines how to go from CPP file to Object file (use %.c* for all files)
	# Compiling $<
	@mkdir -p $(@D)
	@$(MAKE) -c $< -o $@ $(OPT);

$(BUILD_FOLDER)/%.o: %.c # This rule defines how to go from CPP file to Object file (use %.c* for all files)
	# Compiling $<
	@mkdir -p $(@D)
	@gcc -g -Wall -DDEBUG -DINFO -D_GLIBCXX_USE_NANOSLEEP -std=c11 $(INC) -c $< -o $@;

clean:
	# Cleaning $(TARGET)...
	@$(RM) -r $(BUILD_FOLDER); # Remove build folder
	@$(RM) -r $(TARGET); # Remove application