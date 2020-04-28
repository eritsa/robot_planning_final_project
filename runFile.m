mex goal_planner.cpp
map = [1 0 0 0 0 0 0 2;
       0 0 0 0 0 0 0 0;
       0 -1 0 0 -1 0 0 0;
       0 0 0 0 0 0 0 0;
       0 0 -1 0 0 0 0 0;
       0 0 0 0 0 -2 0 0;
       0 0 0 -2 0 0 0 0;
       1 0 0 0 0 0 0 2];
robotpos = [0,0];
depletion = [0.2, 0.2, ...
             0.1, 0];
action = goal_planner(map, robotpos, depletion)