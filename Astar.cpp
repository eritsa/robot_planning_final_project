#include <iostream>
#include <fstream>
#include <bits/stdc++.h> 
#include <chrono>



int const xsize = 1000;
int const ysize = 1000;


bool isValid(int x, int y, double** obs, int x_size, int y_size){
    if (x<0 or y<0 or x>=x_size-1 or y>=y_size-1){
        
        return false;
    }
    if(obs[x][y] == 1){
        return false;
    } else {
        return true;
    }
}



class point{
    private:
        int x;
        int y;
        double g;
        double eps;
    public: 
        point(int _x, int _y, double _eps){
            x = _x;
            y = _y;
            g = INT32_MAX;
            eps = _eps;
        }

        void setG(int _g){
            g = _g;
        }
        int getX(){
            return x;
        }
        int getY(){
            return y;
        }
        double getG(){
            return g;
        }
        double getF(point goal){
            return (abs(x-goal.getX())+abs(y-goal.getY()))*eps + g;

        }
        

};

const int ndir = 8;

double g[xsize][ysize];
bool closed[xsize][ysize];
int dX[ndir] = {-1, -1, -1,  0,  0,  1, 1, 1};
int dY[ndir] = {-1,  0,  1, -1,  1, -1, 0, 1};


point *start;// = new point(0,0);
point *goal;// = new point(10,20);



std::vector<std::vector<int>> plan(int x0, int y0, int x1, int y1, int x_size, int y_size, double** obs){
memset(g, 0, xsize*ysize*sizeof(int));
memset(closed,0,xsize*ysize*sizeof(bool));
auto startTime = std::chrono::steady_clock::now();
double eps = 2;
start = new point(x0,y0,eps);
goal  = new point(x1,y1,eps);
// double g[xsize][ysize];
// bool closed[xsize][ysize];
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
    while(!goalReached and !open.empty()){
        point p = open.top();
        open.pop();
        closed[p.getX()][p.getY()]=true;
        //std::cout<<"("<<p.getX()<<", "<<p.getY()<<")"<<std::endl;
        if (p.getX() == goal->getX() and p.getY() == goal->getY()){
            goalReached == true;
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
                    point *newP = new point(potx,poty,eps);
                    newP->setG(g[p.getX()][p.getY()]+cost);
                    g[potx][poty]=g[p.getX()][p.getY()]+cost;
                    open.push(*newP);

                }  
            }
        }
    }
    auto midTime = std::chrono::steady_clock::now();
    
//     for (int r = 0; r<xsize; r++){
//     for (int c = 0; c<ysize; c++){
//         if (g[r][c]==DBL_MAX){
//         std::cout<<"100,      ";}
//         else{
//         std::cout<<g[r][c]<<", ";
//         }
//     }
//     std::cout<<std::endl;
// }


    std::vector<std::vector<int>> out;
    
    std::vector<int> next;
    next.push_back(goal->getX());
    next.push_back(goal->getY());
    bool found = false;
    while (found == false){
        if ((next[0]==start->getX() and next[1]==start->getY())){
            found == true;
            break;
        }
        out.emplace(out.begin(),next);
        int x;
        int y;
        double thisg = DBL_MAX;
        for (int i = 0; i<ndir; i++){
            // std::cout<<dX[i]<<","<<dY[i]<<std::endl;
            if (isValid(next[0]+dX[i],next[1]+dY[i],obs,x_size,y_size)){
                if (g[next[0]+dX[i]][next[1]+dY[i]]<thisg){
                    x = next[0]+dX[i];
                    y = next[1]+dY[i];
                    thisg = g[x][y];

                    // std::cout<<x<<","<<y<<std::endl;
                }
            }
        }
        
        next.clear();
        next.push_back(x);
        next.push_back(y);
        // std::cout<<next[0]<<","<<next[1]<<std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();

    std::cout<< "Time through A*:" << std::chrono::duration_cast<std::chrono::milliseconds>(midTime - startTime).count()<<" ms"<<std::endl;
    std::cout<< "Time through Backtrace:" << std::chrono::duration_cast<std::chrono::milliseconds>(endTime - midTime).count()<<" ms"<<std::endl;
    return out;

}
// int main(){
    
//     std::cout<<"HI"<<std::endl;
//     start = new point(0,0);
//     goal  = new point(2300,5000);
//     auto p = plan();
//     // for (auto it1 = p.begin(); it1<p.end(); it1++){
//     //     for (auto it2 = it1->begin(); it2<it1->end(); it2++){
//     //         std::cout<<*it2<<",";
//     //     }
//     //     std::cout<<std::endl;
//     // }

//     return 0;
// }