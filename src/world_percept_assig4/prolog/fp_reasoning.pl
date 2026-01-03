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

% Declaring these predicates as dynamic means that their facts can be modified at runtime using assertz and retractall.
:- dynamic seen/1.
:- dynamic pose/4.
:- dynamic type/2.
:- dynamic visited/1.
:- dynamic not_found/2.
:- dynamic task/1.
:- dynamic home_pose/3.
:- dynamic prob/3.     
% Reserved for Task 2 (learning): after the learning phase, probabilities can be written in the form prob(Target, Place, P).
% -----------------------------
% 1) KB maintenance / assertions
% -----------------------------

% reset_kb/0: resets the knowledge base, making it possible to start “learning from scratch”
% retractall removes all facts that match the given pattern
reset_kb :-
    retractall(seen(_)),
    retractall(pose(_,_,_,_)),
    retractall(type(_,_)),
    retractall(visited(_)),
    retractall(not_found(_,_)),
    retractall(task(_)),
    retractall(home_pose(_,_,_)).

% set_task/1: sets the current task (only one task is kept at any time)
% For example: set_task(find_and_point(bowl)).
set_task(T) :-
    retractall(task(_)),
    assertz(task(T)).

% set_home_pose/3: sets the home reference pose (only one is kept)
% For example: set_home_pose(0.0, 0.0, 0.0).
set_home_pose(X,Y,Yaw) :-
    retractall(home_pose(_,_,_)),
    assertz(home_pose(X,Y,Yaw)).

% assert_percept/4: a unified fact-injection interface intended to be called from the C++ side
% Inputs: Name, X, Y, Z
% Functionality:
% 1) asserts seen(Name)
% 2) updates pose(Name, X, Y, Z)
% 3) infers a coarse type T based on name rules and then asserts type(Name, T)
% Name normalization: supports 'bowl' / bowl / "bowl"
normalize_name(NameIn, NameOut) :-
( string(NameIn) -> atom_string(NameOut, NameIn)
; atom(NameIn) -> NameOut = NameIn
).

assert_percept(NameIn, X, Y, Z) :-
normalize_name(NameIn, Name),

% seen/1
( seen(Name) -> true
; assertz(seen(Name))
),

% pose/4: always maintains the most up-to-date value
retractall(pose(Name,_,_,_)),
assertz(pose(Name,X,Y,Z)),

% type/2: inferred from name-based rules
infer_type_from_name(Name, T),
retractall(type(Name,_)),
assertz(type(Name, T)).

percept4(Name, X, Y, Z) :-
seen(Name),
pose(Name, X, Y, Z).

% mark_searched/2: records that a given Place has been searched but the Target was not found

mark_searched(Target, Place) :-
    % visited(Place): indicates that the location has been visited
    ( visited(Place) -> true ; assertz(visited(Place)) ),
    % not_found(Target,Place): indicates that the target was explicitly not found at the given place
    ( not_found(Target, Place) -> true ; assertz(not_found(Target, Place)) ).

% -----------------------------
% 2) Name -> type heuristics (EDIT THIS)
% -----------------------------

% infer_type_from_name/2: performs coarse type classification based on the name string
% Adapt these rules to the actual Gazebo object_name naming scheme; otherwise, type inference will be unreliable.
% Examples: table_big, table_small, shelf1, trash_bin, bowl1, etc.
% Note: substring matching is implemented via sub_atom/5.

% If the name contains "bowl", then the inferred type is bowl
infer_type_from_name(Name, bowl)    :- atom(Name), sub_atom(Name, _, _, _, bowl), !.

% If the name contains "table", then the inferred type is table
infer_type_from_name(Name, table)    :- atom(Name), sub_atom(Name, _, _, _, table), !.

% If the name contains "shelf" or "bookshelf", then the inferred type is shelf
infer_type_from_name(Name, shelf)    :-
    atom(Name),
    (sub_atom(Name, _, _, _, shelf); sub_atom(Name, _, _, _, bookshelf)),
    !.

% If the name contains "bin" or "trash", then the inferred type is trashbin
infer_type_from_name(Name, trashbin) :-
    atom(Name),
    (sub_atom(Name, _, _, _, bin); sub_atom(Name, _, _, _, trash)),
    !.

% If none of the above patterns match, the type is set to unknown
infer_type_from_name(_, unknown).

% -----------------------------
% 3) Inference predicates (Task1-1)
% -----------------------------

% (I1) Semantic abstraction: mapping from physical type to functional category
% A table belongs to the class of support surfaces (support_surface)
support_surface(X) :- type(X, table).

% A bookshelf belongs to the category of storage
storage(X)         :- type(X, shelf).

% A trash bin belongs to the category of container
container(X)       :- type(X, trashbin).

% (I2) Checking whether the target has already been observed: if the task is find_and_point(Target)
% and there exists some seen(Obj) such that type(Obj, Target) holds, then the target is considered found
target_seen(Target, Obj) :-
    seen(Obj),
    type(Obj, Target).

% target_found/2: currently equivalent to target_seen; defined separately for future extensions
% For example, one may later require proximity to the target or satisfaction of additional attributes
% (e.g., color) before considering the target as found
target_found(Target, Obj) :-
    target_seen(Target, Obj).

% (I3) Candidate search location inference: a bowl is more likely to be on a table (support_surface)
% This reflects prior knowledge: even without observing the bowl, the system can infer where to search
candidate_place(bowl, Place) :- support_surface(Place).



% (I4) Whether exploration is required (fallback inference)
% Conditions:
%  1) the target has not been found
%  2) there is no known candidate location in the current knowledge base
%     i.e., there does not exist (candidate_place(Target, P), seen(P))
need_explore(Target) :-
    task(find_and_point(Target)),
    \+ target_found(Target, _),
    \+ (candidate_place(Target, P), seen(P)).

% (I5) Exclusion inference: if a location has been visited and is explicitly marked as not_found,
% then this location is excluded from future consideration
ruled_out(Target, Place) :-
    visited(Place),
    not_found(Target, Place).

% -----------------------------
% 4) Decision predicates (Task1-3)
% -----------------------------

% decide_search_order/2: determines the search order (outputs the list PlacesOrdered)
% First, collect candidate locations Candidates (requirements: candidate and already seen)
% If Candidates is non-empty:
%  - if learning-based probabilities prob/3 are available, sort by probability
%  - otherwise, sort using a default strategy (prioritizing unvisited locations)
decide_search_order(Target, PlacesOrdered) :-
    findall(P, (candidate_place(Target, P), seen(P)), Candidates),
    Candidates \= [],
    (   has_probabilities(Target, Candidates)
    ->  sort_by_prob_desc(Target, Candidates, PlacesOrdered)
    ;   default_order(Target, Candidates, PlacesOrdered)
    ), !.

% If there are no candidate locations, the resulting order is empty
decide_search_order(_, []).

% decide_next_goal/2: determines the next location to visit
% Selects the first Place in the search order that has not been excluded
decide_next_goal(Target, Place) :-
    decide_search_order(Target, Places),
    member(Place, Places),
    \+ ruled_out(Target, Place),
    !.

% If no suitable location exists, return none (to be used by decide_action as a fallback)
decide_next_goal(_, none).

% decide_action/2: final high-level action output (the most critical decision predicate)
% Logic:
% 1) if the target is found -> return_home_then_point(Obj)
% 2) otherwise, if exploration is required -> explore(scan_rotate)
% 3) otherwise, if a next goal exists -> navigate_to(Place)
% 4) otherwise (fallback) -> explore(patrol_waypoints)
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

% has_probabilities/2: checks whether at least one location in Candidates
% has an associated probability prob(Target, Place, P)
has_probabilities(Target, Places) :-
    member(P, Places),
    prob(Target, P, _), !.

% sort_by_prob_desc/3: sorts locations in descending order of probability
% If a location has no associated prob value, it is treated as 0.0
sort_by_prob_desc(Target, Places, PlacesSorted) :-
    findall(P-Score,
        ( member(P, Places),
          ( prob(Target, P, Score) -> true ; Score = 0.0 )
        ),
        Pairs),
    keysort(Pairs, Asc),           
    reverse(Asc, Desc),            
    pairs_keys(Desc, PlacesSorted).% Extract the list of place names

% default_order/3: default ordering strategy (without learning)
% Fresh: not visited and not excluded
% Old: visited but not excluded
% Output = Fresh first, followed by Old
default_order(Target, Places, Ordered) :-
    findall(P, (member(P, Places), \+ visited(P), \+ ruled_out(Target, P)), Fresh),
    findall(P, (member(P, Places), visited(P), \+ ruled_out(Target, P)), Old),
    append(Fresh, Old, Ordered).

% -----------------------------
% 6) Debug helpers (optional)
% -----------------------------

% print_kb_summary/0: prints a summary of the current knowledge base, for debugging purposes
print_kb_summary :-
    findall(O, seen(O), Seen),
    findall(O-T, type(O,T), Types),
    findall(P, visited(P), Visited),
    format("Seen: ~w~n", [Seen]),
    format("Types: ~w~n", [Types]),
    format("Visited: ~w~n", [Visited]).