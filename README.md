# Maze Navigation System fork

The goal of this update is to improve the **runtime stability and executability** of navigation tasks in maze environments.  
In the previous version, Markov localisation was implemented using odometry and infrared sensors, with path replanning triggered when localisation confidence dropped. In mazes with complex structures, however, the amount of observable information was often insufficient for the belief to converge reliably. As a result, the system frequently oscillated between localisation correction and replanning within local regions, leading to rapid path switching, visible robot jitter, and unstable task completion. In complex mazes, the robot was generally unable to finish the navigation task.

To address these issues, this version introduces two major changes:  
(1) replacing the source of localisation information, and  
(2) restructuring and strengthening the map construction and path execution pipeline.

## Localisation Source and Structure

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

