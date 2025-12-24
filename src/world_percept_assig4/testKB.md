group 21: Wei Peng, Qiqi Lin
evn: psl

# world_percept_assig4
source ../knowrob_noetic/devel/setup.bash
catkin_make
# Terminal 1: launch reasoning.launch and call Prolog, load yaml + launch reasoning_node，and generates the query file
source devel/setup.bash
roslaunch world_percept_assig4 reasoning.launch

# Terminal 2: launch Gazebo + perception + map
source ../knowrob_noetic/devel/setup.bash
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/student/ros/workspaces/ssy236_qiqil/src/world_percept_assig4
source devel/setup.bash
roslaunch world_percept_assig4 gazebo_ssy236.launch

# Terminal 3: keyboard navigation allows the robot to see certain objects
source ../tiago_public_ws/devel/setup.bash
rosrun key_teleop key_teleop.py
# colse all terminal,the new object should already store in TXT file.

# test if our KB can read the TXTfile
# Terminal 1: 
source devel/setup.bash
roscore
# Terminal 2:
source devel/setup.bash
roslaunch world_percept_assig4 reasoning.launch
# Terminal 3:
source devel/setup.bash
rosparam load /home/student/ros/workspaces/ssy236_qiqil/src/world_percept_assig4/config/loadKnowledge.yaml
rosrun world_percept_assig4 knowledge_node $(rospack find world_percept_assig4)

# Terminal 4:
source devel/setup.bash
rosservice call /load_knowledge "start: 1"
#  Now in Terminal 3:
you can see it loaded up.
# Now in Terminal 4:
rosservice call /rosprolog/query 0 "check_bookshelf2" "getClassPath('bookshelf', CP), rdf(CP, rdf:type, owl:'Class')"
we can see the bookshelf class is true


# remember clean the TXT file for next time test the write of TXT File.

