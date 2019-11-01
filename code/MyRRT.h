#ifndef MYRRT_H
#define MYRRT_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <cstdlib>
#include <cmath>
#include <climits>
#include "planner.h"

using namespace std;

double GETDISTANCE(vector<double>& n1, vector<double>& n2);

class MyRRT{
    private:
    double* RRTMap;        // Map.
    int dim;               // Dimension of the configuration space.
    int x_size;
    int y_size;
    int num_pathpts;       // Number of points along the planned path.
    double epsilon = 1;    // Epsilon initialization.
    double threshold = 0.001; // Threshold that determines whether reach the goal.
    int max_num = INT_MAX; // Maximum number of nodes in the RRT.
    vector<double> goal;   // Goal point in C space.
    vector<double> start;  // Start point in C space.

    public:
    vector<vector<double>> RRTNode;  // Contains every node of the RRT. ID: i ==> Node: RRTNode[i].
    vector<vector<double>> path;     // Contains every node of the planned path.
    unordered_map<int, int> parent;  // Stores backtracking information.
    int num;               // Number of nodes.
    MyRRT () {}
    MyRRT(int DOF, vector<double>& s, vector<double>& g, double* map, int Xsize, int Ysize){ 
        dim = DOF; 
        start = s;
        goal = g;
        RRTMap = map;
        x_size = Xsize;
        y_size = Ysize;
        RRTNode.push_back(start);    // Push the start node to the RRTNode.
        num = 1;                     // Number of nodes.
        parent[0] = 0;               // Set the parent of start node to be itself.
        num_pathpts = 0;
    }
    
    void set_epsilon(double val);
    void set_threshold(double val);
    void set_maxnum(int val);
    vector<double> samplePts();      // Sample points to the RRT.
    vector<double> getNearest(vector<double>& n1, int& nearest);       // Get the nearest RRT node of given node.
    vector<double> getSampled(vector<double>& n1, vector<double>& n2); // Use epsilon to get the final node to be sampled.
    bool checkCollision(vector<double>& n1, vector<double>& n2);       // Check if there is collision between n1 and n2.
    void planning(double*** plan, int& numofsamples);                  // Plan a path from start to goal.

};
#endif