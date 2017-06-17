# AWP Trajectory Planning and Control
We address the problem of autonomous trajectory planning for **aerial work platforms** (AWP), which are traditionally operated by human technicians but whose safety and efficiency would largely benefit from automation.

Our approach combines several optimal algorithms into an elegant, efficient and highly generic motion planner which is able to handle manipulators and obstacles in arbitrary configurations. We design a simple controller and perform realistic simulations of a 2-DOF arm using `MATLAB`.

<p align="center">
	<img src="doc/plots/sim1.gif" width=“90%"/>
</p>

## Approach

Given a set of known obstacles, a start and a goal positions, our solution computes the shortest collision-free path and trajectory that meet some dynamic constraints, and simulates the resulting dynamic behaviour of the robot:

1. Compute the collision-free configuration space (C-space) and workspace (W-space)
2. Use PRM to compute the shortest path in the C-space
3. Prune the resulting path
4. Generate a smooth trajectory composed of linear segments with parabolic blends
5. Simulate the arm using a PID controller with gravity compensation

An extensive description is available in `doc/report.pdf`.

## Usage
A graphical user interface is provided for easy interaction with the model and the algorithms. The official `Robotics System Toolbox` is required. The workflow is as follows:

* run the `MATLAB` script `launcher.m`
* wait until the C-space and W-space are generated (it may take some time)
* when the cursor turns into crosshairs, chose and click at a target point either in the C-space or in the W-space
* the waypoints of the piecewise linear path (red) as well as its pruned version (blue) are shown in the C-space
* the controller is simulated and an animation of the movement is shown in the W-space
* once the animation is finished, the user can press the `enter` key to open a dialog box and chose to either plan a new motion or exit the simulation.

The simulation and the interface are highly configurable through various parameters that can be tuned in `launcher.m`, including:

* GUI display: show the results of the different stages of the processing pipeline, such as the PRM nodes, waypoints, animation, etc…
* Kinematic and dynamic model of the robot: links lengths and masses, initial joint positions, maximum joints velocity and acceleration, etc…
* Environment: modify or add new obstacles (limited to circular obstacles)
* Algorithms: tune the parameters for the pathfinding, trajectory generation, PID controller gains, etc…

## Credits
The project was developed by Nicolai Domingo Nielsen and Paul-Edouard Sarlin, both exchange students at the National University of Singapore for the academic year 2016-2017.

