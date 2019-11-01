#include <iostream>
#include "MyRRT.h"

using namespace std;

class RRT_Connect{
    private:
    MyRRT RRT_s;
    MyRRT RRT_g;

    public:
    RRT_Connect() {}
    RRT_Connect(int DOF, vector<double>& s, vector<double>& g, double* map, int Xsize, int Ysize){
        MyRRT temp_start (DOF, s, g, map, Xsize, Ysize);
        MyRRT temp_goal (DOF, g, s, map, Xsize, Ysize);
        RRT_s = temp_start;
        RRT_g = temp_goal;
    }
    
    bool connect(MyRRT& A, vector<double>& pts);
    void swapRRT(MyRRT& A, MyRRT& B);
    void planning(double*** plan, int& numofsamples);
};