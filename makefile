# Makefile for Swarmulator
#
# Mario Coppola, 2017-2018

TARGET = swarmulator # application name
BUILD_FOLDER = build
SRC_FOLDER = sw

# Compiler parameters
# Thanks to the help from
# https://www.cs.swarthmore.edu/~newhall/unixhelp/howto_makefiles.html#creating
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings
CC = g++ # chosen compiler
CFLAGS += -g -Wall -std=gnu++0x -DDEBUG -DINFO
OPT=-lglut -lGLU -lGL -lXi -lXmu -lglfw -lpthread -lxerces-c -Wno-deprecated-declarations -fno-inline-functions

# General parameters to include all cpp files and all subfolders
INC_DIRS = $(shell find sw -maxdepth 50 -type d) # Max depth 50 layers. Should be enough.
INC_PARAMS = $(foreach d, $(INC_DIRS), -I$d) #Each include folder must have a -I before it
INC = -I. -I$(SRC_FOLDER) -I$(BUILD_FOLDER) $(INC_PARAMS) # All include paths
SOURCES = $(shell find $(SRC_FOLDER) -name *.cpp) # Recursively find all cpp files
MAKE = $(CC) $(CFLAGS) $(INC)
OBJECTS=$(SOURCES:.cpp=.o)

# Build the executable
# Using @...; suppresses the output of the arguments
all: $(TARGET)

$(TARGET): xsd $(OBJECTS)
	# Building target
	@$(MAKE) $(BUILD_FOLDER)/*.cxx $(OBJECTS) -o $@ $(OPT);

xsd:
	# Generating parameters XSD file
	@mkdir -p $(BUILD_FOLDER);
	@xsd cxx-tree --output-dir "$(BUILD_FOLDER)" --root-element-all conf/parameters.xsd;

%.o: %.cpp # This rule defines how to go from CPP file to Object file
	# Compiling $<
	@$(MAKE) -c $< -o $@ $(OPT);

clean:
	# Cleaning $(TARGET)...
	@$(RM) -r $(BUILD_FOLDER); # Remove build folder
	@$(RM) -r $(TARGET) $(OBJECTS); # Remove application