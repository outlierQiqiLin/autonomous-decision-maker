# world_percept_assig4
source ../knowrob_noetic/devel/setup.bash
catkin_make

# Terminal 1: launch Gazebo + perception + map
source ../knowrob_noetic/devel/setup.bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/student/ros/workspaces/ssy236_qiqil/src/world_percept_assig4
source devel/setup.bash
roslaunch world_percept_assig4 gazebo_ssy236.launch

# Terminal 2: keyboard navigation allows the robot to see certain objects
source ../tiago_public_ws/devel/setup.bash
rosrun key_teleop key_teleop.py

# Terminal 3: launch tiago_control_node
source devel/setup.bash
rosrun world_percept_assig4 tiago_control_node

# Terminal 4: launch GotoObject (take bookshelf as an example)
rosservice call /GotoObject "{start: 1, object_name: 'bookshelf'}"


you can see the robot automatic move to the bookshelf and stop in front of it.