

bins = [0,0]
while length(bins)<21
    ind = randi([50,1000-50],1,2);
    if map(ind(1),ind(2)) == 0
        bins(end+1,1:2) = ind;
    end
end
bins = bins(2:end,:);

bins = [bins,randi([1,4],20,1), ones(20,1)]

machines = [24,24,1,.5;
    999-25,24,2,.75;
    999-25,999-25,3,.2;
    24,999-25,4,1;]

save('initialconditions','bins','machines')