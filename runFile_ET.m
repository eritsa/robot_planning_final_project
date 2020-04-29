mex goal_planner.cpp
map = [0 0 1 1 1 1 0 0 ;
       0 0 0 0 0 0 0 0 ;
       1 0 1 1 0 1 1 0 ;
       1 0 0 1 0 1 1 0 ;
       1 0 0 1 0 0 1 0 ;
       1 0 0 0 0 0 1 0 ;
       0 0 0 0 0 0 1 0 ;
       0 0 0 0 0 0 0 0 ];
% map = zeros(8,8);
map = zeros(1000,1000);

map(5:2:995, 5:995) = 1;
%%
robotpos = [0,0];

machine = [0, 0, 1, 1; 
           0, 999, 2, 0.8;
           999, 0, 3, 1;
           999, 999, 4, 1];
       
warehouse = [1, 2, 1, 1; 
             4, 2, 1, 1;
             2, 4, 1, 1;
             7, 127, 2, 1;
             125, 977, 2, 1;
             201, 951, 2, 1;
             203, 365, 2, 1];


% depletion = [0, 0.05, ...
%              0.1, 0];
[xpath, ypath, waypoints] = goal_planner(map, robotpos, machine, warehouse);
path = [xpath;ypath];