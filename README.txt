TO RUN:
	- In a terminal run:
	$ roslaunch turtlebot_stage turtlebot_in_stage.launch
	- Then go to file open, and navigate to your git hub folder and pick: custom_Config.rviz

	- In another terminal run:
	$ python turtlebot.py
	The walls should get bigger and you should see them change to red on the map. 
	the system will print: Map is none...\n\r Map is instantiated Properly\n\rRobot Created

	At which point you may add 2D Nav Goals




What the code has:
	Turtlebot:
		Contains:
			Init node
			map (from class map)
			local map (from class local map)
			name :: string
			x and y offsets :: int? float? Not sure what they are
			_wheelbase == width of robot


			Publishers
				Velocity Publisher :: 'cmd_vel_mux/input/teleop' Twist

			Subscriber
				Odom subscriber :: '/odom' <type?> odomCallback
				click subscriber :: '/move_base_simple/goalRBE' PoseStamped, self.storeGoal
				local costmap subscriber ::  '/move_base/local_costmap/costmap' OccupancyGrid, self.storeCostmap
			TF
				odom_list :: listener
				odom_tf :: Transform Broadcaster


		Methods:
			Callback Methods:
				storeGoal(...)
				storeCostmap(...)
				odomCallback(...)
			Move Commands
				_publishTwist
				_stopRobot
				_spinWheels

				driveStraight(speed, dist)
				rotate(angle, speed)
				driveTo(point)
				run()
				main()
	Map:
		Contains:
			current_x, current_y, current_theta :: location of the robot in map space
			lots of lists of nodes for displaying in RVIZ

			Publishers:
				_walls					:: '/walls' 			GridCells
				_explored_nodes			:: '/explored_nodes' 	GridCells
				_not_explored_nodes		:: '/waypoints' 		GridCells
				_path					:: '/path'				GridCells
			Subscribers
				_map_sub 				:: '/map' 				OccupancyGrid	:: self._updateMap

		Methods:
			_start_populate(...)
			_updateMap
			_repaint_map(...)
			_updateLocation()

			isAtGoalPosition(...)
			isAtGoalAngle(...)
			getGoalAngle()
			getRobotPosition()
			getRobotAngle()

			getNextWaypoint()
			storeGoal(...)
			getNeighbors(...)
			aStarSearch(..)
			getWaypoint()
			getPath(..)

			main()




What the code does:
	The robot can be configured so that if a point is drawn on the map, the robot can drive to it.
		- This does not take into account changes in the local map - it is essentially part 1 of the lab.
		- If the robot was given close enough waypoints, it could potentially drive the whole path in a non-dynamic environment
			- The concern here is that the drive commands are very bad and inaccurate. This is a known bug and is not a problem.
	Part 2 of the lab: costmap stuff
		- the robot has a subscribing node to the costmap in turtlebot: _local_cost this is the costmap for the robot's immediate surroundings (the local map)
			- This frame is in the odom's frame, therefore there is a need to transform the frame from the global frame to the costmap frame
			- Or, transform from the costmap frame to the robot frame. Either way it needs to happen.
			- One solution would be to use the transformation to just place the obsticals on the global map and refresh it often.
	The rest of the lab revolves around proving that this works on the real robot.
		- I'm unsure of how gmapping changes what the robot sees, but in a local space vs. global space we the '/map' topic
		  holds a bunch of information about the surroundings and the local costmap does a similar thing, but fo the local area.
		  If we are able to get the data to update based off the local costmap, it would be possible to apply that to the gmapping
		  demo, and then I think it's done.
	updating Heuristic:
		- Not sure what they want with this, however our heuristic should start to take into account the cost of the world around it.
			OR
		- We screen out what is below a threshold and dynamicaly modulate the threshold based off what it sees.
		:: Meaning:
		- The robot can make decisions on if it wants to travel to places with 99 value there
			or
		- The map screens out places of 20 value and if a path can't be found ups that threshold to 30, then if a path can't be found to 40.
			Doing it this way would guarentee the 'safest' path at all times, and possibly an equally fast path. However safest != fastest

UPDATE: 11/30/2015 :: 2AM:
	- We now have the local map displaying on the global map in rViz. 
	BUGS: the map is displacing the old values. So, in map.addValue there should be some latency. So if the value was set to 100 and then gets set to 0, it should be weery of just dropping so quicly. Maybe average them together each time to protect form errors and rapid changes. 
	MOVING FORWARD: We need to update the Heuristic to use the cost values so that it stays in 'the center' of the world while it moves. 
	TEST WITH GMapping: I'm not sure how this will port over to GMapping. We need to work on this. 
	OBSTICAL EXPANDING: Already exists by using the costmap. 
	MOVEMENT: Will be bad but we have to do it to get the signoffs.
	WAYPOINTS: We need to chop our waypoints into 1 meter waypoints. They are currently not. 

