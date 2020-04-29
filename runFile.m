clear, clc, close all
%mex goal_planner.cpp
% map = [1 0 0 0 0 0 0 2;
%        0 0 0 0 0 0 0 0;
%        0 -1 0 0 -1 0 0 0;
%        0 0 0 0 0 0 0 0;
%        0 0 -1 0 0 0 0 0;
%        0 0 0 0 0 -2 0 0;
%        0 0 0 -2 0 0 0 0;4,4
%        1 0 0 0 0 0 0 2];
robotpos = [0,0];
%depletion = [0, 0.05, ...
             0.1, 0];

             
depletionRate = .001;
repletionRate = .001;
load('map.mat')
load('initialconditions.mat')
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


%todo whileloop, generate plan, output bin and machine positions as well as action list etc.

%while 1==1         
    %action = goal_planner(map, robotpos, depletion)
action = [1:1000;1:1000]';
binpos = [24,24];
binnum = 4;


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
        bins(binnum,4) = bins(binnum,4)-.2;
    end
    
    
    %update supply levels in machines and bins
    for i = 1:length(bins)
        if bins(i,4)<1-repletionRate
            bins(i,4) = bins(i,4)+repletionRate;
        end
    end
    for i = 1:length(machines)
        if machines(i,4)>depletionRate
            machines(i,4) = machines(i,4)-.depletionRate;
        end
    end
    %update drawing
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

%%    %draw the positions
if (hr ~= -1)
        delete(hr);
        delete(ht);
end
hr = text(robotpos(1), robotpos(2), 'R', 'Color', 'g', 'FontWeight', 'bold');
ht = text(targetpos(1), targetpos(2), 'T', 'Color', 'm', 'FontWeight', 'bold');

%end
