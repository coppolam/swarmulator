# Thanks to the help from
# https://www.cs.swarthmore.edu/~newhall/unixhelp/howto_makefiles.html#creating
#  -g    adds debugging information to the executable file
#  -Wall turns on most, but not all, compiler warnings

APP = swarmulator

CFLAGS  += -std=c++0x

VPATH=animation:simulation

$(info ************ SWARMULATOR V0.1 **********)

# Build an executable named main from main.c
all: 
	g++ -g -Wall -std=gnu++0x \
	-o $(APP) \
	main.cpp \
	draw.cpp \
	particle.cpp \
	agent.cpp \
	controller.cpp \
	omniscient_observer.cpp \
	txtwrite.cpp \
	randomgenerator.c \
	-lglut -lGLU -lGL -lXi -lXmu -lglfw

clean: 
	$(RM) swarmulator
