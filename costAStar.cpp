#include <iostream>
#include <fstream>
#include <bits/stdc++.h> 
#include <chrono>
#include "Astar.cpp"

double get_distance(int x0, int y0, int x1, int y1, int x_size, int y_size, double** obs){

auto startTime = std::chrono::steady_clock::now();
memset(g, 0, xsize*ysize*sizeof(int));
memset(closed,0,xsize*ysize*sizeof(bool));
start = new point(x0,y0);
goal  = new point(x1,y1);
// double g[x_size][y_size];
// bool closed[x_size][y_size];
int dX[ndir] = {-1, -1, -1,  0,  0,  1, 1, 1};
int dY[ndir] = {-1,  0,  1, -1,  1, -1, 0, 1};
for (int r = 0; r<xsize; r++){
    for (int c = 0; c<ysize; c++){
        g[r][c] = DBL_MAX;
        closed[r][c] = false;
    }
}

    class CompareG 
    { 
    public: 
        int operator() (point& p1, point& p2) 
        { 
            return p1.getG() > p2.getG(); 
        } 
    }; 

    class CompareF 
    { 
    public: 
        int operator() (point& p1, point& p2) 
        { 
            return p1.getF(*goal) > p2.getF(*goal); 
        } 
    };

    std::priority_queue <point, std::vector<point>, CompareF > open;

    start->setG(0);
    g[start->getX()][start->getY()] = 0;
    closed[start->getX()][start->getY()] = true;
    open.push(*start);
    bool goalReached = false;
    double distance;
    while(!goalReached and !open.empty()){
        point p = open.top();
        open.pop();
        closed[p.getX()][p.getY()]=true;
        //std::cout<<"("<<p.getX()<<", "<<p.getY()<<")"<<std::endl;
        if (p.getX() == goal->getX() and p.getY() == goal->getY()){
            goalReached == true;
            distance = g[p.getX()][p.getY()];
            break;
        }

        for (int i = 0; i<ndir; i++){
            int potx = p.getX()+dX[i];
            int poty = p.getY()+dY[i];
            
            double cost; //assume cost is one if straight, sqrt(2) if diag
            if (abs(dX[i]*dY[i])==1){
                cost = sqrt(2);
            }
            else{
                cost = 1;
            }
            if (isValid(potx,poty,obs,x_size,y_size)) {
                
                //std::cout<<g[potx][poty]<<", "<<g[p.getX()][p.getY()]<<std::endl;

                if (g[potx][poty]>g[p.getX()][p.getY()]+cost and closed[potx][poty]==false){
                    //std::cout<<"added to open"<<std::endl;
                    point *newP = new point(potx,poty);
                    newP->setG(g[p.getX()][p.getY()]+cost);
                    g[potx][poty]=g[p.getX()][p.getY()]+cost;
                    open.push(*newP);

                }  
            }
        }
    }
    return distance;
}
