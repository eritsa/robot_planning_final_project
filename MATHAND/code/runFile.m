clear, clc, close all
mex goal_planner.cpp
%% Init Robot/Bin Properties
robotpos = [0,0]; 
depletionRate = .0001;
repletionRate = .00005;

%% Load in map data
load('map.mat')
load('initialconditions2.mat')
figure('units','normalized','outerposition',[0 0 1 1]);
imagesc(map'); axis square; colorbar; colormap jet; hold on;


machinehandles = {}
for i= 1:length(machines)
    machinehandles{end+1} = text(machines(i,1)+1,machines(i,2)+1,['M',num2str(machines(i,3))],'Color',1-[machines(i,4),1-machines(i,4),1],'FontWeight','bold');
end

binhandles = {}
for i = 1:length(bins)
    binhandles{end+1} = text(bins(i,1)+1,bins(i,2)+1,['B',num2str(bins(i,3))],'Color',1-[bins(i,4),1-bins(i,4),1],'FontWeight','bold');
    %draw(bins(i,1),bins(i,2),'Color',1-[bins(i,4),1-bins(i,4),1]);
end
hr = text(robotpos(1)+1,robotpos(2)+1,'R','Color', 'm')

%% Init result vectors
elapsed_time = [];
opened = [];
count = 0;

%% Run planner loop (currently set to 10 iterations)
while(count < 10)
    count = count + 1;
    time = tic;
    [xplan, yplan, waypoints, output_time] = goal_planner(map, robotpos, machines, bins);
    %elapsed_time = [elapsed_time; [toc(time), output_time(1), output_time(2)]];
    elapsed_time = [elapsed_time; [output_time(3), output_time(1), output_time(2)-output_time(1)]];
    opened = [opened, output_time(4)];
    fprintf("Mean elapsed total time %d ms, Iteration: %d\n", [mean(elapsed_time(:,1)), count]);
    fprintf("Mean elapsed task time %d ms, Iteration: %d\n", [mean(elapsed_time(:,2)), count]);
    fprintf("Mean elapsed path time %d ms, Iteration: %d\n", [mean(elapsed_time(:,3)), count]);
    fprintf("Task planner nodes opened %d, Iteration: %d\n", [mean(opened), count]);
    action = [xplan;yplan]';
    % action = [1:1000;1:1000]';
    % binpos = [24,24];
    binpos= [waypoints(3), waypoints(4)];
    index = intersect(find(bins(:, 1) == waypoints(3)),find(bins(:,2) == waypoints(4)));
    binnum = index;
    % binnum = 4;
    
    
    for j = 1:length(action)
        %update position
        robotpos = action(j,:);
        
        %check if legal motion
        if map(robotpos(1)+1,robotpos(2)+1) == 1
            throw(MException('runFile:invalidMove','next move, position %d,%d does not exist',robotpos(1)+1,robotpos(2)+1))
        end
        %TODO check if at bin location and change machine states
        if robotpos(1)==binpos(1) &&robotpos(2)==binpos(2)
            fprintf('picking up from bin B%d at position (%d,%d)\n',[bins(binnum,3),robotpos(1)+1,robotpos(2)+1])
            robot_carrying = bins(binnum,4);
            bins(binnum,4) = 0;
        end
        
        
        %update supply levels in machines and bins
        for i = 1:length(bins)
            if bins(i,4)<1-repletionRate
                bins(i,4) = bins(i,4)+repletionRate;
            end
        end
        for i = 1:length(machines)
            if machines(i,4)>depletionRate
                machines(i,4) = machines(i,4)-depletionRate;
            end
        end
        %update drawing (currently not drawing anything)
        if(mod(j,10)== 0)
            delete(hr);
            hr = text(robotpos(1)+1,robotpos(2)+1,'R','Color', 'm');
            for i= 1:length(machines)
                delete(machinehandles{i});
                machinehandles{i} = text(machines(i,1)+1,machines(i,2)+1,['M',num2str(machines(i,3))],'Color',1-[machines(i,4),1-machines(i,4),1],'FontWeight','bold');
            end
            
            for i = 1:length(bins)
                delete(binhandles{i});
                binhandles{i} = text(bins(i,1)+1,bins(i,2)+1,['B',num2str(bins(i,3))],'Color',1-[bins(i,4),1-bins(i,4),1],'FontWeight','bold');
                %draw(bins(i,1),bins(i,2),'Color',1-[bins(i,4),1-bins(i,4),1]);
            end
            drawnow
        end
    end
    index = intersect(find(machines(:, 1) == waypoints(5)), find( machines(:,2) == waypoints(6)));
    index;
    fprintf('picking up from bin B%d at position (%d,%d)\n',[machines(index,3),robotpos(1)+1,robotpos(2)+1])
    min([machines(index,4)+robot_carrying,1]);
    machines(index,4) = min([machines(index,4)+robot_carrying,1]);
end
close all;