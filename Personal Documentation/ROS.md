## This documentation is gonna be for the gps task

### Description of the task
We have to use the group project robot and implement gps sensor on it to make it move from one coordinate to other assuming that one obstacle is in the path. SO i shall with learning about how sensors actually working in ros.(As we have already learned how to add sensor int he gazebo model of the robot)

### Sesnors and Sensor Messages
So when we plug in a sensor to our pc, it continously sends signal to our machine or terminal in my case. So my first line of thought is to publish those signlas in the form of data to ross topic.

Then gps sensor of our rover shall subscribe to the odom topic to check if the current position matches with the desired position or not, and based on that our rover can be programmed ot move in one or more direction.

So my approach is as following:-

#### Part 1 Making the robot move using a node file

I will be making a node that is capable of moving the robot without me needing to adding the movement buttons.

I am implementing a type of `teleport_node` where it subscribes to the `cmd_vel` topic.

`test1` contains it's implementation is as follows:- 
`odom_sub` subscribes to `/cmd_vel` and gets the message
the shared pointer to it `(msg)` extracts the current position

`target_sub` subscribes to `/target_cmd` and determines how far to move depending on current position

`move_to_target()` is called

checks the difference between current and target position and determines if it should move or not and how much it should move
and publishes it to `cmd_vel_pub`

But this approach only sets the velocities intially and moves it infinitely.
Now I need to change it to move to a limited coordinate.

### Part 2 making the robot 2

Now for this i have thought of a new approach. Initially the plan was to make it in one axis, stop it.
turn it 90 degrees and then move it in the other axis.
But what if I rotate it to face the point, then move the distance equivalent to the net distance between the two points.


    Special stuff
    Using the `RCLPP_LOG` method
    
    We 
    
