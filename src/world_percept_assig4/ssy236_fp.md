# group 21: Wei Peng, Qiqi Lin
# evn: psl

# FP, T01 Reasoning

## R1: Inference Predicates
- `support_surface/1`
- `storage/1`
- `container/1`
- `candidate_place/2`
- `need_explore/1`
- `ruled_out/2`
- `target_seen/2`
- `target_found/2`

## R2: Knowledge Update via Assertions
- `set_task/1`
- `set_home_pose/3`
- `assert_percept/4`
- `mark_searched/2`
- `reset_kb`

## R3: Decision Predicates
- `decide_search_order/2`
- `decide_next_goal/2`
- `decide_action/2`

---

## T01 Testing

### 1) Start the Environment
To start the environment, run the following command:
```bash
rosrun rosprolog rosprolog world_percept_assig4
```

### 2) Validate Inference Ability
Scenario: The robot has just started up and sees a large table(table_big). we need to verify whether it knows that a bowl might be on the table.

?- reset_kb.
true.

?- assert_percept(table_big, 2.0, 2.0, 0.0).
true.

?- support_surface(X).
X = table_big.

?- candidate_place(bowl, Place).
Place = table_big.

### 3) Validate Decision Ability
Scenario: The task is to find a bowl and verify whether the robot has decided to go to the table it just saw.

?- set_task(find_and_point(bowl)).
true.

?- decide_next_goal(bowl, Goal).
Goal = table_big.

?- decide_action(bowl, Action).
Action = navigate_to(table_big).

### 4) Validate Assertions & Logic Update
Scenario: The robot went to the table but didn't find the bowl. we need to verify whether the robot will rule out the table.

?- mark_searched(bowl, table_big).
true.

?- decide_next_goal(bowl, Goal).
Goal = none.

?- decide_action(bowl, Action).
Action = explore(patrol_waypoints).


Results can be found in the 'results' folder 


# FP. T02: Learning

## Description
The objective of this task is to enable the Tiago robot to autonomously navigate through candidate locations to find a target object (e.g., a `bowl`). The system combines prior knowledge, reasoning, and online learning to optimize the search process.

**Workflow:**
1.  **Prior Knowledge:** The system initializes with prior knowledge (loaded from `savedQueries.txt`) to identify potential candidate locations (e.g., `table` and `cafe_table`).
2.  **Initial State:** Initially, all candidate locations are assigned equal probability. The robot selects the search order based on a default sorting mechanism.
3.  **Online Learning:** Based on the outcome of the search (Success or Failure at a specific location), the system updates the probability weights for that location using an online learning approach.
4.  **Optimization:** In subsequent missions, the robot prioritizes locations with higher learned probabilities, effectively reducing the time required to find the target.


## R1: Implement at least one learning approach
- We implemented an online probabilistic learning approach based on Bayesian updating (Beta–Bernoulli model). This approach allows the robot to update the probability of finding objects at specific locations after each observation.

## R2: Use at least one online tool
- We used Pickle for persistent data management to store learned probabilities across sessions.

## R3:  Implement at least one evaluation performance
- We use 'Confusion Matrix' to evaluate performance which can be found in the 'results' folder.

Changes of probability before and after learning and Confusion Matrix can be found in the 'results' folder

---

## T02 Testing Instructions

Follow these steps to validate the Learning and Navigation loop.

### 1) Start the Simulation Environment (terminal1)
Launch the Gazebo world with the Tiago robot.

source ../knowrob_noetic/devel/setup.bash
catkin_make
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/student/ros/workspaces/ssy236_qiqil/src/world_percept_assig4
source devel/setup.bash
roslaunch world_percept_assig4 gazebo_ssy236.launch

### 2) Start the Reasoning System (terminal2)
Launch the Prolog reasoning engine to handle logic and inference.

source devel/setup.bash
roslaunch world_percept_assig4 reasoning.launch

### 3) Start the Knowledge Interface (terminal3)
Start the node responsible for interfacing with the Knowledge Base (handling assertions and queries).

source devel/setup.bash
rosrun world_percept_assig4 knowledge_node $(rospack find world_percept_assig4)

### 4) Start the Online Learning Module (Python) (terminal4)
Important: For the first run (or to reset learning), delete the memory file to start with a clean state.

Optional: Run this command ONLY if you want to clear previous learning results:
rm ~/.ros/ml_prob_memory.pkl

source devel/setup.bash
cd ./src/world_percept_assig4
chmod +x scripts/ml_online_learning_node.py
rosrun world_percept_assig4 ml_online_learning_node.py

### 5) Start the Control & Mission Loop (terminal5)
Start the main C++ node that coordinates reasoning, learning, and robot movement.

source devel/setup.bash
rosrun world_percept_assig4 learning_node

### 6) Load Prior Knowledge (terminal6)
Inject the initial facts (e.g., location of tables) into the Knowledge Base to trigger the candidate generation.

source devel/setup.bash
rosservice call /load_knowledge "start: 1"

