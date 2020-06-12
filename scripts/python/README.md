### Install requirements

Make sure you have python3 installed.
To install the necessary sub-packages run:

    pip3 install -r requirements.txt

# Main
 - `evolution_example.py`. This is an example file that evolves an aggregation controller for Swarmulator. Try it out and then use it as a baseline to make your own evolution!

    To run it use the command:
        python3 evolution_example.py aggregation particle

    You can replace aggregation and particle with the controller and agent of your choice!

#### Classes
 - `evolution.py`. This is a wrapper around the DEAP python library for the main commands.
 You can edit it in order to change the way that the evolution is run.

 - `simulator.py`. This is a swarmulator API to command and interface with swarmulator from Python.


