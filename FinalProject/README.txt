Final Project Flowchart:


Startup:
    - Spin in a circle to get a general map of surroundings (done with the startupSpin command)
    - From here, the robot should search it's map to find a frontier
        -- Ideally the robot is maintaining a list of ALL the frontiers that it sees so that it doesn't miss any.
Operation
    - To be sure that the robot is not setting points in 'unknown space' the robot should place it's 'setpoints' in
        the points midway between the frontier and the robot.
        -- The robot should use the 'driveTo' command to pick it's place to go. Once there, it can spin in a
            circle to see more of it's surroundings if needed.
    - From there, the robot will begin to move towards the location that was set.
        -- At this point the robot will monitor the /move_base/result topic and listen for error messages and arrival messages
        -- This will use the '_nextAction' method which will queue up what to do next.
            This should be done by setting flags that the main loop responds to.
    - The robot will loop these actions until the robot can no longer find any more frontiers
    - At this point the robot will be considered done.



What needs to be done:
    Startup:
        - The functionality for startup is mostly done, the startupSpin command is written.
        - TODO: in the __init__ of turtlebot, we need to pic a value for the startupSpinVelocity
    Frontier Exploration:
        - There needs to be a function to find the frontiers that exist on the map.
            - TODO: Write frontier Expansion algorithm
        - This algorithm could be apart of the turtlebot class or the map class, either one works.
    General:
        - TODO: write the control loops for catching 'incorrect' base messages and handle them acccordingly
            -- This means that if the robot makes it to its destination it should pick a new location on that frontier
            -- This means that if the robot realizes it cannot make it to the location that you have asked it to go to
                it should catch that error and pick a new frontier to go to
    Possible Problems:
        - The robot can get stuck in a state where it spins in a circle and keeps trying to figure out what to do next.
            This error needs to get caught but until we know what the topics are doing during that we have no way to know
            how to catch it. When we do catch it, the robot should just stop what it's doing and make a new goal and drive to it.
        - We have to be careful not to spawn off too many threads while we are working on this. Each time a callback is run a
            thread is spawned off. This means that we need to make sure that the robot is dealing with these threads and then
            letting them die and then the robot respons to the new information in main loop.
        - There is a possibility that hte robot could miss a frontier beacuse our algorithm is too 'cheep' early. In the case
            where parts of the map are not filled in we need to be able to catch this somehow. 