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

robotpos = [0,0];

machine = [0, 0, 1, 1; 
           0, 7, 2, 0.8;
           7, 0, 1, 1;
           7, 7, 2, 1];
warehouse = [1, 2, 1, 1; 
             4, 2, 1, 1;
             2, 4, 1, 1;
             5, 5, 2, 1;
             6, 3, 2, 1];


% depletion = [0, 0.05, ...
%              0.1, 0];
[xpath, ypath, waypoints] = goal_planner(map, robotpos, machine, warehouse);