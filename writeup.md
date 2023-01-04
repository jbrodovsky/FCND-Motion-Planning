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

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.


And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.


Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.



### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.

