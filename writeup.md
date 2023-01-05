# Project: 3D Motion Planning Writeup
![Quad Image](./misc/enroute.png)

---

This project integrated the basic path planning and event based programming from the previous project with a set of more robust path-planning algorithms.

## Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points

### Explain the Starter Code

#### 1. Verify that `motion_planning.py` and `backyard_flyer_solution.py` work

I ran both in the Motion Planning simulator. `backyard_flyer_solution.py` works as intended and is a replica of the previous project. The drone uses event based programming to fly to an altitude of 3 meters and then executes a flight plan for a 10 by 10 meter square before returning to its original location and landing. `motion_planning.py` has a similar function. Instead of a squaure flight plan, this directs the drone to follow a staircase pattern roughly due northeast.

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

`motion_planning.py` is another implementation of the drone similar to what was originally done in the Backyard Flyer project. However, this time the class was slightly more generalized to allow for any waypoint trajectory and transtions between arbitray waypoints as provided by the planning algorithm. The method `plan_path` replaces the original path planner the calculated a basic square path and will be what I fill in. At project start it provides a basic staircase pattern.

`planning_utils.py` contains the actual algorithms and tools for generating the discrete occupancy grid, how we define valid actions, and the A* planning algorithm.

### Implementing Your Path Planning Algorithm

To meet the objectives for an 8-connected A* route planner I modified the `Action` class in `planning_utils.py` and the `plan_path` method from the `MotionPlanning` class in `motion_planning.py`.

#### 1. Set your global home position

I used basic file IO to open `colliders.csv` and read the first line. I then used the `strip` and `split` functions to isolate the individual strings for the latitude and longitude. I then used `split` again to find the numerical value and cast it to a float.

```
with open('colliders.csv') as f:
    line0 = f.readline()
line0 = line0.strip()
lat_str, lon_str = line0.split(',')
lat0 = float(lat_str.split()[1])
lon0 = float(lon_str.split()[1])
```

And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position

These previous latitude and longitude coordinates with an altitude of zero were used to set the drone's home position.

```
self.set_home_position(lon0, lat0, 0)
```

#### 3. Set grid start position from local position

Using the `create_grid` function to get the raw data grid as well as the relative north and east offsets I calculated the grid start coordinate relative to the current location:

```
grid_start = (int(current_local_position[0]) - north_offset,
              int(current_local_position[1]) - east_offset)
```

#### 4. Set grid goal position from geodetic coords
Using manual control I flew the drone around to get suitable goal position coordinates. I used the `global_to_local` function transform these coordinates to the local grid frame and added in the grid start coordinates. Due to a bug, the buildings and environment did not render in my simulator, so I added an extra check to make sure that the grid cell was not occupied.

```
goal_lon = -122.397010
    goal_lat = 37.792970
    lon_goal, lat_goal, _ = global_to_local((goal_lon, goal_lat, 0), current_global_position)
    grid_goal = (int(lon_goal) + grid_start[0], int(lat_goal) + grid_start[1])
    if grid[grid_goal[0], grid_goal[1]]:
        print('Goal Cell occupied!')
        return
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)

I added in four additional values to the `Action` class:

```
NORTHEAST = (1, 1, np.sqrt(2))
NORTHWEST = (1, -1, np.sqrt(2))
SOUTHEAST = (-1, 1, np.sqrt(2))
SOUTHWEST = (-1, -1, np.sqrt(2))
```

I also modified the `valid_action` method to check these additional actions:

```
if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
    valid_actions.remove(Action.NORTHEAST)
if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
    valid_actions.remove(Action.NORTHWEST)
if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
    valid_actions.remove(Action.SOUTHEAST)
if x + 1 < 0 or y - 1 < 0 or grid[x + 1, y - 1] == 1: 
    valid_actions.remove(Action.SOUTHWEST)
```

#### 6. Cull waypoints 

To cull unneccessary waypoints, I used the collinearity test developed in the lessons (`prune_path`) which uses two addtional private methods: `_point` and `_collinearity_check`. This method takes an array-like datastructure containing (x,y,z) points and checks to see if intermediary points are collinear.

### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
Rubric points are met.