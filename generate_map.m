clear, clc, close all
xsize = 1000;
ysize = 1000;
map = zeros(xsize,ysize);
for i = 1:xsize
    for j = 1:ysize
        if mod(i,20)==0% || mod(i,10)==1 || mod(i,10)==2
            
            if ~(mod(j,50) == 0 || mod(j,50)==1 || mod(j,50)==2|| mod(j,50)==3|| mod(j,50)==4)
                 map(i,j) = 1;
            else
                map(i,j) = round(rand);
            end
        end
        
        
        
    end
end

mapE = zeros(xsize,ysize);
for i = 1:xsize
    for j = 1:ysize
        if map(i,j) == 1
            for x = i-1:i+1
                for y = j-1:j+1
                    if x>0 && x<xsize &&y>0 && y<ysize
                        mapE(x,y) = 1;
                    end
                end
            end
        end
    end
end

        


mapE(1:50,1:50) = 0;
mapE(end-50:end,1:50)=0;
mapE(end-50:end,end-50:end) = 0;
mapE(1:50,end-50:end) = 0;



imshow(-1*mapE+1)


mapEA = mapE;
mapEA(:,ysize/2-5:ysize/2+5) = 0;
figure
imshow(-1*mapEA+1)

fprintf("number of impassable walls: %d\n",sum(sum(mapE')>=xsize-2))



f1 = fopen('warehouse.txt','w+')


for i = 1:xsize
    for j = 1:ysize
        fprintf(f1,'%d  ',mapE(i,j));
        
    end
    fprintf(f1,'\n');
end
fclose(f1);

f = fopen('warehouse_with_aisle.txt','w+')
for i = 1:xsize
    for j = 1:ysize
        fprintf(f,'%d  ',mapEA(i,j));
        
    end
    fprintf(f,'\n');
end
fclose(f);