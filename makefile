# Thanks to the help from
# https://www.cs.swarthmore.edu/~newhall/unixhelp/howto_makefiles.html#creating
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings

TARGET = swarmulator
CC = g++
CFLAGS += -g -Wall -std=gnu++0x -DDEBUG -DINFO

OPT=-lglut -lGLU -lGL -lXi -lXmu -lglfw -lpthread -lxerces-c -Wno-deprecated-declarations

BUILD_FOLDER = build
SW_FOLDER = sw
SIMULATION_FOLDER = $(SW_FOLDER)/simulation
ANIMATION_FOLDER = $(SW_FOLDER)/animation
LOGGER_FOLDER = $(SW_FOLDER)/logger
MATH_FOLDER = $(SW_FOLDER)/math

INC=-I. -I$(SW_FOLDER) -I$(BUILD_FOLDER) -I$(SIMULATION_FOLDER) -I$(ANIMATION_FOLDER) -I$(SIMULATION_FOLDER)/agents -I$(SIMULATION_FOLDER)/controllers  -I$(LOGGER_FOLDER) -I$(MATH_FOLDER) 

# Build the executable
# Using @ suppresses the output of the arguments
all: 
	@echo "Generating parameters parser...";
	@mkdir -p $(BUILD_FOLDER);
	@xsd cxx-tree --output-dir "$(BUILD_FOLDER)" --root-element-all conf/parameters.xsd;
	@echo "Building $(TARGET)...";
	@$(CC) $(CFLAGS) $(INC) -o $(TARGET) $(SW_FOLDER)/main.cpp $(BUILD_FOLDER)/*.cxx  $(SIMULATION_FOLDER)/agents/*.cpp $(SIMULATION_FOLDER)/controllers/*.cpp $(SIMULATION_FOLDER)/*.cpp $(ANIMATION_FOLDER)/*.cpp $(LOGGER_FOLDER)/*.cpp $(MATH_FOLDER)/*.cpp $(OPT);
	@echo "Done";

clean:
	@echo "Cleaning $(TARGET)...";
	@$(RM) -r $(BUILD_FOLDER);
	@$(RM) -r $(TARGET);
	@echo "Done";