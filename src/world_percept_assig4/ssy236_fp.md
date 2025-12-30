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

# FP. T03: Robotics
