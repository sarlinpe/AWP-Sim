# ME5402/EE5106R ADVANCED ROBOTICS — Project 1-2: Trajectory Planning

The whole simulation runs on `MATLAB`. It was developed with `R2016b` but should be working on previous versions. The official `Robotics System Toolbox` is required.

A graphical user interface is provided for easy interaction with the model and the algorithms. The workflow is as follows:

* run the `MATLAB` script `launcher.m`
* wait until the configuration space (C-space) and workspace (W-space) are generated (it may take some time)
* when the cursor turns into crosshairs, chose and click at a target point either in the C-space or in the W-space
* the waypoints of the piecewise linear path (red) as well as its pruned version (blue) are shown in the C-space
* the controller is simulated and an animation of the movement is shown in the W-space
* once the animation is finished, the user can press the `enter` key to open a dialog box and chose to either plan a new motion or exit the simulation.



The simulation and the interface are highly configurable through various parameters that can be tuned in `launcher.m`, including:

* GUI display: show the results of the different stages of the processing pipeline, such as PRM nodes, waypoints, animation, etc…
* Kinematic and dynamic model of the robot: links lengths and masses, initial joint positions, maximum joints velocity and acceleration, etc…
* Environment: modify or add new obstacles (limited to circular obstacles)
* Algorithms: tune the parameters for the pathfinding, trajectory generation, PID controller gains, etc…