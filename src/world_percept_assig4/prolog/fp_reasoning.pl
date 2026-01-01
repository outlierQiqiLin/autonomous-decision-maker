/*  fp_reasoning.pl
 *
 *  Template for SSY236 Final Project - Task1 (Reasoning)
 *
 *  - Facts asserted at runtime (from perception):
 *      seen/1, pose/4, type/2
 *  - Process facts:
 *      visited/1, not_found/2, task/1, home_pose/3
 *
 *  - Inference predicates (>=4): 
 *      target_seen/2, target_found/2, support_surface/1, storage/1, container/1,
 *      candidate_place/2, need_explore/1, ruled_out/2
 *
 *  - Decision predicates (>=3):
 *      decide_search_order/2, decide_next_goal/2, decide_action/2
 *
 *  Notes:
 *    1) This is a reference template: adapt naming rules and regions/places.
 *    2) Works even with partial knowledge: fallback to explore.
 */

% 声明这些谓词是“动态的”，意味着运行时可以 assertz/retractall 修改它们的事实
:- dynamic seen/1.
:- dynamic pose/4.
:- dynamic type/2.
:- dynamic visited/1.
:- dynamic not_found/2.
:- dynamic task/1.
:- dynamic home_pose/3.
:- dynamic prob/3.     % 给 Task2 (learning) 预留：学习后可以写入概率 prob(Target,Place,P)

% -----------------------------
% 1) KB maintenance / assertions
% -----------------------------

% reset_kb/0：重置知识库，使“从0认知开始”成为可能
% retractall 会删除所有匹配的事实
reset_kb :-
    retractall(seen(_)),
    retractall(pose(_,_,_,_)),
    retractall(type(_,_)),
    retractall(visited(_)),
    retractall(not_found(_,_)),
    retractall(task(_)),
    retractall(home_pose(_,_,_)).

% set_task/1：设置当前任务（只保留一个 task）
% 例如：set_task(find_and_point(plate)).
set_task(T) :-
    retractall(task(_)),
    assertz(task(T)).

% set_home_pose/3：设置home原点位置（只保留一个）
% 例如：set_home_pose(0.0,0.0,0.0).
set_home_pose(X,Y,Yaw) :-
    retractall(home_pose(_,_,_)),
    assertz(home_pose(X,Y,Yaw)).

% assert_percept/4：建议由 C++ 侧调用的“统一事实注入入口”
% 输入：Name, X, Y, Z
% 功能：
%  1) 断言 seen(Name)
%  2) 更新 pose(Name,X,Y,Z)
%  3) 通过名字规则推断一个粗类型 T，然后断言 type(Name,T)
assert_percept(Name, X, Y, Z) :-
    % 如果已经 seen 过则不重复 assert；否则写入 seen(Name)
    (   seen(Name) -> true ; assertz(seen(Name)) ),

    % pose 更新策略：先删旧的 pose(Name,...) 再写入新的
    retractall(pose(Name,_,_,_)),
    assertz(pose(Name,X,Y,Z)),

    % 从名字推断类型（这一步你必须按 Gazebo 模型命名调整）
    infer_type_from_name(Name, T),

    % 同样：type 只保留一个，先删再写
    retractall(type(Name,_)),
    assertz(type(Name, T)).

% mark_searched/2：记录“我搜索过某个 Place，但没找到 Target”
% 这是“负信息”，非常重要：即使没有新物体出现，你也能积累新知识
mark_searched(Target, Place) :-
    % visited(Place)：表示去过/访问过该地点
    ( visited(Place) -> true ; assertz(visited(Place)) ),
    % not_found(Target,Place)：表示在该地点明确没找到目标
    ( not_found(Target, Place) -> true ; assertz(not_found(Target, Place)) ).

% -----------------------------
% 2) Name -> type heuristics (EDIT THIS)
% -----------------------------

% infer_type_from_name/2：基于名字字符串进行类型粗分类
% 你必须根据 Gazebo 里实际 object_name 命名做修改，否则 type 会乱
% 例如：table_big, table_small, shelf1, trash_bin, plate1 等
% 注意：这里用的是 sub_atom/5 做“包含子串”的匹配

% 如果名字包含 "bowl" 则类型=bowl
infer_type_from_name(Name, bowl)    :- atom(Name), sub_atom(Name, _, _, _, bowl), !.

% 如果名字包含 "table" 则类型=table
infer_type_from_name(Name, table)    :- atom(Name), sub_atom(Name, _, _, _, table), !.

% 如果名字包含 "shelf" 或 "bookshelf" 则类型=shelf
infer_type_from_name(Name, shelf)    :-
    atom(Name),
    (sub_atom(Name, _, _, _, shelf); sub_atom(Name, _, _, _, bookshelf)),
    !.

% 如果名字包含 "bin" 或 "trash" 则类型=trashbin
infer_type_from_name(Name, trashbin) :-
    atom(Name),
    (sub_atom(Name, _, _, _, bin); sub_atom(Name, _, _, _, trash)),
    !.

% 都不匹配则 unknown
infer_type_from_name(_, unknown).

% -----------------------------
% 3) Inference predicates (Task1-1)
% -----------------------------

% (I1) 语义抽象：从物理类别 type 推到“功能类别”
% 桌子属于可放置表面（support_surface）
support_surface(X) :- type(X, table).

% 书架属于收纳处（storage）
storage(X)         :- type(X, shelf).

% 垃圾桶属于容器（container）
container(X)       :- type(X, trashbin).

% (I2) 目标是否已经看到：如果任务是 find_and_point(Target)
% 且存在一个 seen(Obj) 且 type(Obj,Target)，则认为目标已看到
target_seen(Target, Obj) :-
    seen(Obj),
    type(Obj, Target).

% target_found/2：这里暂时和 target_seen 等价，分开写是为了以后扩展
% 例如你未来可以要求：必须靠近目标、或必须满足颜色等条件才算 found
target_found(Target, Obj) :-
    target_seen(Target, Obj).

% (I3) 候选搜索点推断：盘子更可能在桌子（support_surface）
% 这是“先验知识”体现：不需要看到盘子也能推断“该去哪找”
candidate_place(bowl, Place) :- support_surface(Place).

% 如果你未来希望“盘子也可能在书架上”，就加：
% candidate_place(plate, Place) :- storage(Place).

% (I4) 是否需要探索（fallback 推断）
% 条件：
%  1) 目标没 found
%  2) 当前知识里没有任何“已知候选地点”
%     即：不存在 (candidate_place(Target,P), seen(P))
need_explore(Target) :-
    task(find_and_point(Target)),
    \+ target_found(Target, _),
    \+ (candidate_place(Target, P), seen(P)).

% (I5) 排除地点推断：如果访问过且明确 not_found，则该地点被排除
ruled_out(Target, Place) :-
    visited(Place),
    not_found(Target, Place).

% -----------------------------
% 4) Decision predicates (Task1-3)
% -----------------------------

% decide_search_order/2：决定搜索顺序（输出 PlacesOrdered 列表）
% 先收集候选地点 Candidates（要求：候选且已 seen）
% 如果 Candidates 非空：
%  - 若存在 learning 的 prob/3，则按概率排序
%  - 否则按默认策略排序（未访问优先）
decide_search_order(Target, PlacesOrdered) :-
    findall(P, (candidate_place(Target, P), seen(P)), Candidates),
    Candidates \= [],
    (   has_probabilities(Target, Candidates)
    ->  sort_by_prob_desc(Target, Candidates, PlacesOrdered)
    ;   default_order(Target, Candidates, PlacesOrdered)
    ), !.

% 如果没有候选地点，则顺序为空
decide_search_order(_, []).

% decide_next_goal/2：决定下一步要去的地点
% 从搜索顺序里选第一个“还没被排除”的 Place
decide_next_goal(Target, Place) :-
    decide_search_order(Target, Places),
    member(Place, Places),
    \+ ruled_out(Target, Place),
    !.

% 如果没有合适地点，返回 none（供 decide_action fallback 使用）
decide_next_goal(_, none).

% decide_action/2：最终高层动作输出（最关键的决策谓词）
% 逻辑：
% 1) 如果找到目标 -> return_home_then_point(Obj)
% 2) 否则如果 need_explore -> explore(scan_rotate)
% 3) 否则如果有 next goal -> navigate_to(Place)
% 4) 否则兜底 -> explore(patrol_waypoints)
decide_action(Target, Action) :-
    task(find_and_point(Target)),
    (   target_found(Target, Obj)
    ->  Action = return_home_then_point(Obj)
    ;   need_explore(Target)
    ->  Action = explore(scan_rotate)
    ;   decide_next_goal(Target, Place),
        Place \= none
    ->  Action = navigate_to(Place)
    ;   Action = explore(patrol_waypoints)
    ).

% -----------------------------
% 5) Helpers for Task2 integration
% -----------------------------

% has_probabilities/2：判断 candidates 中是否至少有一个地点有 prob(Target,Place,P)
has_probabilities(Target, Places) :-
    member(P, Places),
    prob(Target, P, _), !.

% sort_by_prob_desc/3：按 prob 值降序排序
% 若某地点没有 prob，则视为 0.0
sort_by_prob_desc(Target, Places, PlacesSorted) :-
    findall(P-Score,
        ( member(P, Places),
          ( prob(Target, P, Score) -> true ; Score = 0.0 )
        ),
        Pairs),
    keysort(Pairs, Asc),           % keysort 按 Score 升序
    reverse(Asc, Desc),            % reverse 得到降序
    pairs_keys(Desc, PlacesSorted).% 取出地点名列表

% default_order/3：默认顺序（无学习时）
% Fresh：没访问过且没排除
% Old：访问过但没排除
% 输出 = Fresh 在前 + Old 在后
default_order(Target, Places, Ordered) :-
    findall(P, (member(P, Places), \+ visited(P), \+ ruled_out(Target, P)), Fresh),
    findall(P, (member(P, Places), visited(P), \+ ruled_out(Target, P)), Old),
    append(Fresh, Old, Ordered).

% -----------------------------
% 6) Debug helpers (optional)
% -----------------------------

% print_kb_summary/0：打印当前 KB 概览，调试用
print_kb_summary :-
    findall(O, seen(O), Seen),
    findall(O-T, type(O,T), Types),
    findall(P, visited(P), Visited),
    format("Seen: ~w~n", [Seen]),
    format("Types: ~w~n", [Types]),
    format("Visited: ~w~n", [Visited]).