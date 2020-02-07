# behavior_tree 

## Description
This controller reads a behavior tree and runs the behavior accordingly.
It used the behavior tree library in the sub-folder `bt`, which was originally written by Kirk Scheper, and has now been edited for inclusion within Swarmulator.

## Usage
Sample behavior trees are defined in: `conf/behavior_trees/`
You can use use your own behavior tree by changing the behavior tree file in use, which is defined in the controller's constructor.

Note that it will only work if you follow the correct xml formatting for the behavior tree.

## Behavior tree xml formatting
A sample behavior tree can look as following:
```
<BTtype>composite<function>sequence<vars><name>sequence<endl>
	<BTtype>condition<function>greater_than<vars>0,0.51<name>robot<endl>
	<BTtype>action<function>wheelSpeed<vars>0.1066,0.1375<name>robot<endl>
```
We have 4 types of nodes that we can use:

- **Action**
An action node is defined by, for example:
`<BTtype>action<function>wheelSpeed<vars>0.1066,0.1375<name>robot<endl>`
The action sets the wheelSpeed in the behavior tree. It can be accessed in the behavior_tree.cpp through
`BLKB.get("wheelSpeed0");`
The command reads the 0th parameter of wheelSpeed (which in the example above = 0.1066)

- **Condition**
A condition node is defined by, for example:
`<BTtype>condition<function>greater_than<vars>0,0.51<name>robot<endl>`
A condition depends on the input to the behavior tree, which we set through 
`BLKB.set("sensor0",0.5);`
This sets the sensor #0 to 0.5.
In the example above, the condition will fail since 0.5<0.51

- **Selector**
A selector node (usually visualized as `?`) is declared as, for example:
`<BTtype>composite<function>selector<vars><name>selector<endl>`
- **Sequence**
A composite node (usually visualized as `!`) is defined as, for example:
`<BTtype>composite<function>sequence<vars><name>sequence<endl>`

## Troubleshoot tips
- Make sure that there are no empty spaces at the end of the behavior tree xml file!