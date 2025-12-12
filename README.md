# Maze Navigation System fork

The goal of this update is to improve the **runtime stability and executability** of navigation tasks in maze environments.  
In the previous version, Markov localisation was implemented using odometry and infrared sensors, with path replanning triggered when localisation confidence dropped. In mazes with complex structures, however, the amount of observable information was often insufficient for the belief to converge reliably. As a result, the system frequently oscillated between localisation correction and replanning within local regions, leading to rapid path switching, visible robot jitter, and unstable task completion. In complex mazes, the robot was generally unable to finish the navigation task.

In further system testing, we observed another issue that is largely independent of theoretical localisation accuracy yet significantly affects task success. In structurally complex mazes or environments with insufficient observations, the robot can exhibit persistent orientation jitter, rapid switching between nearby local paths, and repetitive small-scale replanning. Although these behaviours do not necessarily indicate a localisation failure, they can undermine trajectory following at the execution level, causing the robot to oscillate back and forth in the maze or even fail to reach the goal.

Motivated by this practical runtime behaviour, this fork focuses on improving operational stability in complex environments through deterministic pose inputs, map reconstruction consistency, and smoother path execution.

To address these issues, this version introduces two major changes:  
(1) replacing the source of localisation information, and  
(2) restructuring and strengthening the map construction and path execution pipeline.

## 1. Localisation Source and Structure

On the localisation side, GPS and IMU yaw data provided by Webots are introduced as global pose inputs. The localiser no longer relies on Markov-style probabilistic updates, but instead directly outputs the robot’s pose in world coordinates ((x, y, \theta)). To maintain compatibility with the existing system architecture, the localiser keeps its original interfaces (such as `update`, `estimate_pose`, and `measure_uncertainty`), while internally replacing the probabilistic belief with a deterministic representation based on the grid cell corresponding to the GPS position. Accordingly, the role of the localisation module shifts from state estimation to pose provision, and the system’s focus moves toward whether planning and control can be executed in a stable manner.

## 2. Map Construction and Visualisation

Rebuilt the grid map automatically from the existing world file so the maze layout used by planning matches the simulation environment. The script converts walls and arena boundaries into a grid representation suitable for navigation, without manually redefining the map. (`build_maze_grid_new.py`)

Added a visualisation step to inspect the generated grid map and wall alignment, making it easy to verify that the parsed geometry matches the actual maze and to catch mapping errors early instead of debugging blindly. (`test_grid.py`)

## 3. Planning and Execution

On the planning and execution side, A* is still used for path search. On top of that, the discrete grid path is smoothed using a uniform cubic B-spline and sampled at a fixed resolution to produce a discrete trajectory that is more suitable for continuous tracking by a differential-drive robot. At the control level, positional and angular dead zones are introduced, along with a waypoint look-ahead mechanism, to reduce oscillation and back-and-forth motion near path nodes.

## 4. Runtime Monitoring and Recovery

For runtime monitoring, the system keeps the overall structure of anomaly detection, recovery, and replanning, but the underlying criteria are changed. In this version, `confidence` is no longer used to represent localisation uncertainty; instead, it serves as a runtime execution status indicator. A new `lost_detector` module compares motion inferred from wheel encoders with changes perceived by infrared sensors to detect wheel slip, getting stuck, or abnormal collisions. When such abnormal conditions persist beyond a predefined threshold, the system triggers a recovery behaviour (backing up and turning) and invokes the replanner. The confidence time series and experiment logs are recorded for later analysis and comparison.

## 5. Experimental Validation

For experimental validation, the updated system is tested on both structurally simple and complex maze maps. A random start position mechanism is introduced to cover navigation behaviour under different initial conditions. The evaluation focuses on runtime stability across varying environment complexity and starting states, rather than optimal paths in a single fixed scenario. (`random_spawn_new.py`)

### 5.1 Stability Evaluation Under Random Initial Conditions (Fork Version)

**Aim.**  
The aim of this fork’s evaluation is not to compare path optimality, but to assess whether the system can reliably execute and complete goal-directed navigation under random initial conditions. In particular, the focus is on avoiding common failure modes such as in-place oscillation, repeated replanning loops, and getting stuck spinning near walls.

**Environment & goal.**  
All experiments were conducted in **Webots R2025a** using the **e-puck** robot model. Unless otherwise stated, the new maze world (`maze_world_new.wbt`) is used as the representative environment. The goal location is fixed near the green GOAL region in the new map, specified in the controller via:
- `set_goal_world(0.52, 0.27)`

A run is considered successful when the Euclidean distance between the robot’s current position and the goal is below **0.05 m**.

**Map selection.**  
To cover different levels of structural complexity, two maze maps were tested: one with a simpler layout and one with a more complex layout. Both maps use the same controller and the same planning–execution pipeline, so the comparison primarily reflects stability and executability across environments rather than controller changes.

**Randomised start state.**  
Before each trial, `random_spawn_new.py` samples a random start position from the walkable region of the occupancy grid and randomises the robot’s initial heading. Sampling is constrained to remain within a closed navigable region, and a minimum obstacle clearance is enforced (`clearance_cells = 1`). The script reads the translation and boundary parameters of the `RectangleArena`, converts sampled grid coordinates into world coordinates, and writes the updated pose back into the `.wbt` file. This ensures that all generated start states remain valid and within the arena bounds.

**Time limit & repetitions.**  
For each map, **30** independent trials were conducted with different random start positions and headings. Each run was capped at **15 minutes** of simulated time. If the robot met the goal condition within the time limit, the trial was marked as a success; otherwise it was recorded as a timeout failure.

**Logged outputs.**  
During execution, the visualisation window provides the planned path, real-time position updates, and a live confidence curve. After each run, the system logs data to CSV files, including:
- Robot pose at each step and distance-to-goal: `experiment_log_adaptive.csv`
- Confidence time series from the lost detector: `confidence_history.csv`

Console output additionally reports whether the goal was reached, along with the step count at completion and the estimated path length.

**Results.**  
Under this setup, the robot successfully reached the goal in **30 out of 30** randomised trials on each of the two maps within the 15-minute limit. These results indicate that the forked version provides improved execution stability under random initial conditions: even when brief contact events or short-lived anomalies occur, the system is able to maintain feasible trajectory-following behaviour and eventually complete navigation.

**Limitations & future work.**  
Despite these improvements, the system is still far from solving the broader challenge of reliable navigation under global localisation failure. Future work should prioritise more accurate localisation and perception models (e.g., refining the infrared observation model, or replacing the IR array with more reliable sensing such as lidar/ultrasonic). Another promising direction is to move beyond fixed, hand-designed control logic by incorporating reinforcement-learning-based policies.

## 6. Summary

Overall, this update makes targeted adjustments to localisation, mapping, planning, and execution based on observed runtime behaviour in simulation, resulting in noticeably more stable navigation in complex maze environments.

---

**Note: This version is set up to work with the `maze_world_new.wbt`.**



## How to test by switching maps or using a random start position

### 1) Update the goal interface in `main_controller.py`

- Old map goal: `compute_top_right_goal(localiser)`
    
- New map goal: `set_goal_world(0.52, 0.27)`
    

Example:

```python
goal = set_goal_world(0.52, 0.27)
```



### 2) Generate the new map and replace the grid map in `localisation.py`

Run `build_maze_grid_new.py` to generate the new map, then replace the grid map used in `localisation.py`.



### 3) Change the random spawn (start position)

You can modify the random start position in `random_spawn_new.py`,  
or directly edit it in `maze_world_new.wbt`.

Example:

```wbt
E-puck {
  translation -0.390131 -0.439433
```

