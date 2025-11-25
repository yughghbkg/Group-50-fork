Group-50
Intelligent Robotics:  Autonomous Robot Navigation with Adaptive Re-Planning After Getting Lost

This project implements an intelligent navigation system for the e-puck robot in Webots.
The robot can detect when it becomes lost, perform probabilistic localisation, and trigger adaptive replanning to reach a goal in a maze.

The project includes four main intelligent behaviours:
- Odometry + Markov Localisation
- Lost-State Detection (sensor–motion consistency check)
- A* Path Planning
- Adaptive Re-Planning when localisation confidence drops

The Code Structure contains:
controllers/
  main_controller/
    main_controller.py
    localisation.py
    lost_detector.py
    path_planner.py
    replanner.py
    

worlds/
  maze_world.wbt

What We Implemented Ourselves
1. Localisation Module (localisation.py) — Yuhang
Custom implementation of:
Differential-drive odometry
Discrete Markov localisation
Grid-based sensor likelihood model using IR sensors
Motion model with diffusion noise
Sensor model estimating expected IR readings from map structure
Uses no external robotics libraries (self-built from scratch)

2. Lost-State Detector (lost_detector.py) — Toluwalope
Fully original implementation of:
Wheel encoder motion analysis
IR sensor environment-change analysis
Confidence scoring between [0,1]
Detection of odometry drift, wheel slip, or sudden reorientation
Trigger flag (is_lost=True) for the replanner
Does not depend on external algorithms or packages


4. Path Planner (path_planner.py) — Zhichao
Custom implementation of:
A* graph search to navigate an occupancy grid
Grid coordinate ↔ world coordinate conversion
Waypoint following logic
Simple emergency collision handling
Built without using external A* libraries

5. Replanner (replanner.py) — Lateefat
Implements:
Dynamic replanning when lost_detector confidence < threshold
Resetting and re-initialising Markov belief
Selecting new local goal when lost
Integrating planner + localiser outputs
Fully hand-written logic

6. Main Controller (main_controller.py) — All Members
Integrates all modules:
Localisation update
Lost detection
Replanning
Movement commands
Includes automatic device sanity-checking

What Pre-Programmed Packages or Code We Used
We only rely on:
Webots Built-In Classes
controller.Robot
Motor and sensor device classes (e.g., Motor, DistanceSensor, PositionSensor)

Standard Webots API for:
Reading encoder values
Reading IR sensors
Setting motor velocities
Python Standard Library: math, collections.deque and sys

All localisation, detection, planning, and re-planning is implemented manually.

How to Run the Project
Open Webots
Load the world file:
worlds/maze_world.wbt

Select the robot → set controller to:
main_controller
Press Run
The console will display:
Localisation updates
Lost detection outputs
Replanning triggers
Current waypoint following


How Intelligent Behaviour Is Demonstrated:
The robot estimates its position probabilistically using IR + odometry.
If sensors disagree with motion → confidence drops.
When confidence < 0.4 → robot declares itself lost.
Replanner generates a new path based on the new estimated position.
Robot follows the new path to reach the goal.
Belief Confidence Visualisation
To evaluate the lost-state detection module, we implemented confidence logging inside lost_detector.py.
At every control step, the system records:
the current time-step index
the robot’s localisation confidence value (0–1)
These values are automatically saved to:
controllers/main_controller/confidence_history.csv
Generating the Confidence Plot
A plotting script is included: plot_confidence.py
Run: python3 plot_confidence.py
This produces a graph showing how localisation confidence changes over time.
This visualisation highlights:
- stable movement (confidence near 1.0)
- drift or inconsistent sensor readings (confidence decreases)
- lost events (confidence < 0.4)
- recovery after backoff + replanning (confidence rises again)
The graph is included in the report as evidence of belief tracking and the robot’s diagnostic intelligence.

This system integrates localisation, diagnostics, and planning — satisfying the module’s requirement for intelligent behaviour in a robot.
