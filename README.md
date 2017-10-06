## Swarmulator
A lightweight C++ simulator designed for simulating swarms.
Swarmulator offers a simple platform to prototype swarm behaviors.

Swarmulator was built and tested on Ubuntu 14.04. At the moment, it has not been proven on any other system.

## Set-up
Download/clone the repository.

### Packages Required
As the animation runs using OpenGl, you will need the following packages prior to installation. Run the following:

	sudo apt-get install libglfw3 libglfw3-dev libglfw-dev 

### Build instructions
Once you have all the packages needed to build Swarmulator, you can build it with:
	
	make clean && make

## Running Swarmulator
### Set up the configuration file
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

### Run
To run Swarmulator, use the following command: 

    .\swarmulator <nagents> <knearest>

Where <nagents> is the number of agents you would like to have at the start of the simulation.  <knearest> is the number of agents that each agent can observe starting with the closest from itself. If <knearest> is unspecified, the agents will be able to observe all other agents. So for instance, to run a simulation with 3 agents which can all observe each other, run:

        .\swarmulator 3 2

### User commands
You can interact with Swarmulator in real time through the animation. It is possible to intuitively drag and zoom within the animation using drag-and-drop and your mouse's scroll wheel. 

Additionally, you can do the following:
* Add a new agent (`a`). This can be done by pressing the `a`, at which point a new agent will be created at the location pointed by the mouse cursor.
* Pause (`p`). The simulation can be paused by pressing the `p` key.
* Resume (`r`). Resume the simulation. Pressing `r` while the simulation is running normally will cause it to pause.
* Step-through (`s`). This will run the simulation for a very small time step and then pause. It can be pressed repeatedly to slowly step through the simulation.
* Re-start the simulation (`n`). This will re-start the simulation.
* Quit (`q`). This will quit swarmulator.

## Prototyping with Swarmulator
### Using your controller
The individual controller is accessible in `sw/simulation/controller.cpp`. Here you can define your rule that you would like to test out.
You can use the functions in `OmniscientObserver` in order to simulate the sensing of other agents as you see fit.

### Using your agent dynamics
The standard version of Swarmulator runs using standard kinematic equations as defined in the `Particle` class. `Particle` is a sub-class of `Agent`.
If you would like to use your own dynamics, then you can create a new sub-class for `Agent` where you can include a model of your choice.