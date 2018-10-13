## Project: 3D Motion Planning
CONDA ACTIVATE FCND
---
# Required Steps:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You are reading it - This is the file - Below I describe how I addressed each rubric point and where in my code each point is handled.

---
### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`

##### CRITERIA DESCRIPTION: 
Test that motion_planning.py is a modified version of backyard_flyer_solution.py for simple path planning. Verify that both scripts work. Then, compare them side by side and describe in words how each of the modifications implemented in motion_planning.py is functioning.

##### CRITERIA SPEC: 
The goal here is to understand the starter code. We've provided you with a functional yet super basic path planning implementation and in this step, your task is to explain how it works! Have a look at the code, particularly in the plan_path() method and functions provided in planning_utils.py and describe what's going on there. This need not be a lengthy essay, just a concise description of the functionality of the starter code.

##### RESPONSE:

Confirmed that motion_planning.py (MP) is a modified version of the backyard_flyer_solution.py (BFS) since their code and resultant outcomes are different.

The BFS script flies the drone in a square shape.
The MP flies the drone in jagged step formation.

Both BFS and MP scripts work.

And here is a lovely picture of the MP script end result - it also shows the path taken.
![motion_planning.py works!](./misc/MP_ScreenShot_ItWorks.png)

I will briefly highly the key differences between the BFS (backyard_fyler_solution.py) and MP (motion_planning.py) .

The MP first of all imports planning_utils.py where all the 'juicy' bits are happening.
MP has an extra drone 'State' called PLANNING, which is utilized after we have 'armed' and 'taken control' of the drone.
The PLANNING phase is where all the magic happens, the plan_path() is called at this flight_stage.
In the plan_path() we set our current location, set our current target/goal location (+10, +10), and plan our flight path from start to goal location --> by creating an array of waypoints in the planning_utils.py a_star() method.  The plan_path() ends by visualizing all the waypoints planned on the actual map for visual reference.

After all the waypoints are created, the TAKE OFF phase is initiated and the MP runs through the various flight phases 'popping' each waypoint off the array as it goes along.
Once all the waypoints are complete, the landing and disarming phases follow.

---
### Implementing Your Path Planning Algorithm

#### 1. Set your global home position

##### CRITERIA DESCRIPTION:
In the starter code, we assume that the home position is where the drone first initializes, but in reality you need to be able to start planning from anywhere. Modify your code to read the global home location from the first line of the colliders.csv file and set that position as global home (self.set_home_position())

##### CRITERIA SPEC:
Here you should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home.

##### RESPONSE:
I created a new local function/method/defintion called getFirstRowOfCSV, it returns an array containing lat0 and lon0
After that I called the following function self.set_home_position(lon, lat, 0)

---
#### 2. Set your current local position

##### CRITERIA DESCRIPTION:
In the starter code, we assume the drone takes off from map center, but you'll need to be able to takeoff from anywhere. Retrieve your current position in geodetic coordinates from self._latitude, self._longitude and self._altitude. Then use the utility function global_to_local() to convert to local position (using self.global_home as well, which you just set)

##### CRITERIA SPEC:
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

##### RESPONSE:
First I got the lat, lon and altitude.
I then set geodetic arrays for current location and home location.
Using the built in function global_to_local to convert from GPS to local NED coords.
PS: I did not set self.global_home

---
#### 3. Set grid start position from local position
##### CRITERIA DESCRIPTION:
In the starter code, the start point for planning is hardcoded as map center. Change this to be your current local position.

##### CRITERIA SPEC:
This is another step in adding flexibility to the start location. As long as it works you're good to go!

##### RESPONSE:
The location of where drone is at the time the automation code is executed, is taken as the grid start position.
It works and we ready to go.

---
#### 4. Set grid goal position from geodetic coords
##### CRITERIA DESCRIPTION:
In the starter code, the goal position is hardcoded as some location 10 m north and 10 m east of map center. Modify this to be set as some arbitrary position on the grid given any geodetic coordinates (latitude, longitude)

##### CRITERIA SPEC:
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

##### RESPONSE:
Created two global variables called: TARGET_LON, TARGET_LAT
I used the global_to_local method to convert to NED.
I then set the grid_goal co-ords using the offset and the NED

---
#### 5. Modify A* to include diagonal motion (or replace A* altogether)
##### CRITERIA DESCRIPTION:
Write your search algorithm. Minimum requirement here is to add diagonal motions to the A* implementation provided, and assign them a cost of sqrt(2). However, you're encouraged to get creative and try other methods from the lessons and beyond!

##### CRITERIA SPEC:
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

##### RESPONSE:
The Action class in planning.utils.py got 4 new additions namely: NW, NE, SW, SE and their associated costs and logic applied in the 'valid_actions' definition/function/method.  I implemented a LuigiHam() that gets called by the a_star() which has method level comments that details some of the thinking.
I did not have time to complete my l_star() - however wrote down the thinking.



---
#### 6. Cull waypoints 
##### CRITERIA DESCRIPTION:
Cull waypoints from the path you determine using search.

##### CRITERIA SPEC:
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. In your writeup, explain the code you used to accomplish this step.

Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

##### RESPONSE:
I took the same collinearity functionaliy from my earlier excercise and included the functions in planning_utils.
The prune_path method got called from motion_planning.py

---
### Execute the flight
#### 1. Does it work?
##### CRITERIA DESCRIPTION: 
This is simply a check on whether it all worked. Send the waypoints and the autopilot should fly you from start to goal!

##### CRITERIA SPEC:
At the moment there is some mismatch between the colliders map and actual buildings in the scene. To ensure success build in a 5+ m safety margin around obstacles. Try some different goal locations. Also try starting from a different point in the city. Your reviewer will also try some random locations so be sure to test your solution! There is no firm constraint or requirement on how accurately you land exactly on the goal location. Just so long as your planner functions as expected.

##### RESPONSE:
It works!
However "long missions" (eg: 2-3 blocks away) gives a timeout error as follows: "ConnectionAbortedError: [WinError 10053] An established connection was aborted by the software in your host machine".

---  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


---

Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)


![Quad Image](./misc/enroute.png)

And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![Top Down View](./misc/high_up.png)

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

