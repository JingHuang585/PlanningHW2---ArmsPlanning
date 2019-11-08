#include "RRT_Connect.h"

bool RRT_Connect::connect(MyRRT& A, vector<double>& pts){
    bool connected = false;
    int nearestID = 0;
    vector<double> nearest_node = A.getNearest(pts, nearestID);
    vector<double> sampled_node = A.getSampled(pts, nearest_node);
    bool collision = A.checkCollision(sampled_node, nearest_node);
    if (!collision){
        A.RRTNode.push_back(sampled_node);
        ++(A.num);
        A.parent[A.num-1] = nearestID;                              // New point's ID is: num - 1, parent's ID is: nearestID.
        if (sampled_node == pts){                                   // If sampled_node equals to pts, means two trees connect with each other.
            connected = true;
        }
    }

    return connected;
}

void RRT_Connect::swapRRT(MyRRT& A, MyRRT& B){
    /*
    Swap RRT A and RRT B. 
    */
    MyRRT C = move(B);
    B = move(A);
    A = move(C);
}

void RRT_Connect::planning(double*** plan, int& numofsamples){
    bool terminate = false;
    int count = 0;
    MyRRT Ta, Tb;
    Ta = RRT_s;
    Tb = RRT_g;
    while (!terminate){
        vector<double> temp_node = Ta.samplePts(); 
        int nearestID = 0;
        vector<double> nearest_node = Ta.getNearest(temp_node, nearestID);       // Get the nearest RRT node of the temporary sampled node.
        vector<double> sampled_node = Ta.getSampled(temp_node, nearest_node);    // Get the sampled RRT node using epsilon.
        bool collision = Ta.checkCollision(sampled_node, nearest_node);
        if (!collision){
            Ta.RRTNode.push_back(sampled_node);
            ++(Ta.num);
            Ta.parent[Ta.num-1] = nearestID;                                     // New point's ID is: num - 1, parent's ID is: nearestID.
            bool is_connected = false;
            is_connected = connect(Tb, sampled_node);                                           // Try to connect Tree b to the new sampled_node.

        }
        swapRRT(Ta, Tb);
    }

}