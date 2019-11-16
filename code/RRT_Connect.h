#ifndef RRT_CONNECT_H
#define RRT_CONNECT_H

#include <iostream>
#include <algorithm>
#include "MyRRT.h"
#include "planner.h"

using namespace std;

class RRT_Connect{
    private:
    MyRRT RRT_s;
    MyRRT RRT_g;
    int swapped; // 0: RRT_s and RRT_g are not swapped. 1: Swapped. 
    vector<vector<double>> path;
    int max_num; // Max number of nodes.
    int num_pathpts; // Number of nodes in planned path.
    int dim;

    public:
    RRT_Connect() {}
    RRT_Connect(int DOF, vector<double>& s, vector<double>& g, double* map, int Xsize, int Ysize){
        dim = DOF;
        MyRRT temp_start (DOF, s, g, map, Xsize, Ysize);
        MyRRT temp_goal (DOF, g, s, map, Xsize, Ysize);
        RRT_s = temp_start;
        RRT_g = temp_goal;
        swapped = 0;
    }
    
    void set_maxnum(int val);
    vector<double> extend();
    bool connect(vector<double>& q_samp);
    void swapRRT();
    void planning(double*** plan, int& numofsamples);
};

#endif