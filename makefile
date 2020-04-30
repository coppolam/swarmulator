# Makefile for Swarmulator
#
# Mario Coppola, 2017-2018

TARGET = swarmulator # application name
BUILD_FOLDER = build
SRC_FOLDER = sw

CONTROLLER?=controller_aggregation
AGENT?=particle

# Compiler parameters
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings

CC = g++ # chosen compiler
CFLAGS = -g -Wall -std=gnu++0x -D_GLIBCXX_USE_NANOSLEEP -DSWARMULATOR -DCONTROLLER=$(CONTROLLER) -DAGENT=$(AGENT)
OPT=-lglut -lGLU -lGL -lpthread -lxerces-c -Wno-deprecated-declarations -fno-inline-functions

ifeq ($(VERBOSE),ON)
CFLAGS += -DVERBOSE
endif

ifeq ($(ANIMATION),ON)
CFLAGS += -DANIMATION
endif

ifeq ($(LOG),ON)
CFLAGS += -DLOG
endif

CTRL_FOLDER = sw/simulation/controllers
AGNT_FOLDER = sw/simulation/agents

# General parameters to include all cpp files and all subfolders
# INC_DIRS = $(shell find $(SRC_FOLDER) -path $(CTRL_FOLDER) -prune -o -type d)
INC_DIRS = $(shell find $(SRC_FOLDER) -type d)
SOURCES_CPP = $(shell find $(SRC_FOLDER) -path $(CTRL_FOLDER) -prune -o -path $(AGNT_FOLDER) -prune -o -name *.cpp -print) # Recursively find all cpp/c files
SOURCES_C = $(shell find $(SRC_FOLDER) -path $(CTRL_FOLDER) -prune -o -path $(AGNT_FOLDER) -prune -o -name *.c -print) # Recursively find all cpp/c files

CTRL_INC = $(shell find $(SRC_FOLDER) -name $(CONTROLLER).cpp -printf '%h\n')
SOURCES_CPP +=  $(shell find $(CTRL_INC) -type f -name *.cpp -print)
SOURCES_C +=  $(shell find $(CTRL_INC) -type f -name *.c -print)
INC_DIRS += $(shell find $(CTRL_INC) -type d)

AGNT_INC = $(shell find $(SRC_FOLDER) -name $(AGENT).cpp -printf '%h\n')
SOURCES_CPP +=  $(shell find $(AGNT_INC) -name *.cpp -print)
SOURCES_C +=  $(shell find $(AGNT_INC) -name *.c -print)
INC_DIRS += $(shell find $(AGNT_INC) -type d)

INC_PARAMS = $(foreach d, $(INC_DIRS), -I$d) # Each include folder must have a -I before it
INC = -I. -I$(SRC_FOLDER) -I$(BUILD_FOLDER) $(INC_PARAMS) # All include paths

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

$(BUILD_FOLDER)/%.o: %.c # This rule defines how to go from C file to Object file (use %.c* for all files)
	# Compiling $<
	@mkdir -p $(@D)
	@gcc -g -Wall -DDEBUG -DINFO -D_GLIBCXX_USE_NANOSLEEP -std=c11 $(INC) -c $< -o $@;

clean:
	# Cleaning $(TARGET)...
	@$(RM) -r $(BUILD_FOLDER); # Remove build folder
	@$(RM) -r $(TARGET); # Remove application
