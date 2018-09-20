# Swarmulator

<img align="left" src="https://raw.githubusercontent.com/coppolam/swarmulator/master/logo.png">

Swarmulator is a lightweight C++ simulator for simulating swarms.
Swarmulator offers a simple platform to prototype swarm behaviors.

## Install Instructions
_Note: Swarmulator was built and tested on Ubuntu 16.04. At the moment, it has not been proven on any other system._

#### Download the repository
Download/clone the repository.
    
    git clone https://github.com/coppolam/swarmulator.git
    cd swarmulator
    
#### Packages Required
Swarmulator runs using OpenGL and XML parsing.
You will need the following packages. Run the following to make sure you have everything you need, installed in order:

    sudo apt-get install freeglut3-dev
    sudo apt-get install libxerces-c-dev
    sudo apt-get install xsdcxx
    sudo apt-get install libeigen3-dev

#### Build instructions
Once you have all the packages needed to build Swarmulator, you can build it with:

	make clean && make

If you want it to build fast, use `make -j', but this will take up more memory.

## Running Swarmulator
Swarmulator can be launched from the terminal. To run Swarmulator, use the following command: 

    .\swarmulator <nagents>

Where <nagents> is the number of agents you would like to have at the start of the simulation.
    .\swarmulator 3

#### Setting up the runtime parameters
Swarmulator uses a configuration file called `parameters.xml` in order to load run-time parameters. These are loaded when Swarmulator starts.

`parameters.xml` is to be found in in the `conf` folder.

Here you can edit all the run-time parameters. These are described below:

* `simulation_updatefreq`: Refresh-rate of the simulation.
* `simulation_realtimefactor`: Simulation real-time factor.

* `window_width`: Height of the animation window (in pixels)
* `window_height`: Width of the animation window (in pixels)
* `scale`: Scale of the Agents drawn
* `mouse_drag_speed`: Sensitivity of the mouse during drag-and-drop actions. Use 1.0 for a 1 to 1 feel.
* `mouse_zoom_speed`: Sensitivity of zoom function to motion of the scroll wheel.
* `animation_updatefreq`: Frame-rate of the animation.

* `logger_updatefreq`: Log-rate. If the logger thread is activated, then this will create a text log at the indicated rate (with respect to the simulation time).
 
#### Adding your own runtime parameters
It is possible to easily add your own runtime parameter to Swarmulator.
This can be done into three quick steps.
1. Open `conf/parameters.xsd`. Here you declare the new parameter as an additional XSD element.
        ```<xs:element name="parameter_name" type="xs:parameter_type"/>```
2. Open `conf/parameters.xml`. Here you define the value of the parameter to be read at runtime.
       ```<new_parameter_name>default value</new_parameter_name>```
That's it! Now you use the parameter anywhere in the code by calling `param->new_parameter_name()`.

#### Interactive user commands
You can interact with Swarmulator in real time through the animation window. It is possible to intuitively move and zoom within the animation using drag-and-drop and your mouse's scroll wheel. 

Additionally, there are a number of keyboard commands for the following:
* Add a new agent (`a`). This can be done by pressing the `a`, at which point a new agent will be created at the location pointed by the mouse cursor.
* Pause (`p`). The simulation can be paused by pressing the `p` key.
* Resume (`r`). Resume the simulation. Pressing `r` while the simulation is running normally will cause it to pause.
* Step-through (`s`). This will run the simulation for a very small time step and then pause. It can be pressed repeatedly to slowly step through the simulation.
* Quit (`q`). This will quit Swarmulator.

## Prototyping with Swarmulator
Swarmulator is built for quick prototyping, making it simple to switch out different controllers and agents in the swarm.
The controller and agent in use are defined in `sw/settings.h`.
Using this approach, you can quickly switch out different controllers and then recompile the code.

#### Using your own controller
All controllers must be a child of the class `Controller`, to be found at `sw/simulation/controller.cpp`. 
You can define a controller as a child class of `Controller`, and then define that you want to use it in `sw/settings.h`.
You can look at the controllers in `sw/simulation/controllers/...` for examples.
You can use the functions in `OmniscientObserver` in order to simulate the sensing of other agents as you see fit.

#### Using your agent dynamics
All agents must be a child of the class `Agent`, to be found at `sw/simulation/agent.cpp`. 
You can define an agent as a child class of `Agent`, and, as for controllers, define that you want to use it in `sw/settings.h`.
You can look at the controllers in `sw/simulation/agents/...` for examples of agents, where you can declare the dynamics of the agents.
