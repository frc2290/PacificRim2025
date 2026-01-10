# PacificRim2025 — Excelsior
FRC season 2025, Team FLYT 2290


This is a quick look-up of code organization and how it works.
This is experimental code that proved itself to be reliable and superior to any previous versions, which is under the main branch.
This experimental code implements several new advanced features, including state machines, state machine coordinator, controller profile manager, and robot-user communication manager through network tables. 

For season 2025, the last offseason game, there were two state machines created to simplify system management and combine under one coordinator. State machines implemented a feature called graph-command, which is a simple program where you create points and your connections, as well as the cost. Each point can have a command placed to run when the graph command is at that point. 
Drive state machines take care of running different driving modes, as well as turning on the autonomous mode and rotation locks. Manipulator state machines take care of setting predefined positions and modes for different types of scoring or/and controlling the robot if manual override is needed, preventing state machine brick where state machines lock themselves from switching states because of an internal error or outside physical influence that caused the robot to become physically stuck. New system allows fairly easy implementation of error handling and stall detection to safely reset the robot, partially implemented through a manual override call. The graph command comes into play when specific moves are needed to avoid any external or internal obstacles to reach any target state. The graph command allows setting the position setpoint and connections, and letting it find the shortest predefined connections as physical robot movements, increasing state switching efficiency.
The state coordinator allows communication between states, which allows controlled sync between systems, minimizing the chance of unknown states as well, and it handles errors and control inputs.

Lastly, one of the most useful features that allowed for the simplification of driver input management was the control profiles manager, which depended on the robot's scoring mode. This feature changed key bindings on the controller to produce different actions, including manual mode override.
