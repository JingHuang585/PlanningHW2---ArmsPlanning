#include "RRT_Connect.h"

void RRT_Connect::set_maxnum(int val){
    max_num = val;
}
vector<double> RRT_Connect::extend(){
    /*
    Extend the RRT_s of the RRT_Connect. If successfully extend it, return
    the sampled node. Otherwise, return a uninitialized node.
    */
    int nearestID = -1;
    vector<double> sampled_node = RRT_s.extend(nearestID);
    if(RRT_s.connect(sampled_node, nearestID)){
        return sampled_node;
    }
    else{
        vector<double> uninitialized_node;
        return uninitialized_node;
    }
}

bool RRT_Connect::connect(vector<double>& q_samp){
    /*
    Find the nearest node in RRT_g to the sampled node and try to connect it with
    the sampled node.
    Args:
        q_samp: Sampled node in RRT_s.
    */
    bool connected = false;
    int nearestID = 0;
    vector<double> nearest_node = RRT_g.getNearest(q_samp, nearestID);
    connected = RRT_g.connect(q_samp, nearestID);
    // bool collision = RRT_g.checkCollision(q_samp, nearest_node);
    // if (!collision){
    //     // Successfully connected two RRT together!
    //     RRT_g.RRTNode.push_back(sampled_node);
    //     ++(RRT_g.num);
    //     RRT_g.parent[RRT_g.num-1] = nearestID;                              // New point's ID is: num - 1, parent's ID is: nearestID.
    //     connected = true;
    // }
    return connected;
}

void RRT_Connect::swapRRT(){
    /*
    Swap RRT A and RRT B. 
    */
    MyRRT C = move(RRT_s);
    RRT_s = move(RRT_g);
    RRT_g = move(C);
    swapped = 1 - swapped; 
}

void RRT_Connect::planning(double*** plan, int& numofsamples){
    bool terminate = false;
    int count = 0;
    int goalID = 0;
    while (!terminate){
        vector<double> sampled_node = extend();
        if (sampled_node.size() > 1){
            if(connect(sampled_node)){
                terminate = true;
            }
        }
        swapRRT();
        ++count;
        if (count >= max_num){
            terminate = true;
        }
    }
    if (swapped == 1){ // If RRT_s and RRT_g is swapped, swap them back.
        swapRRT();
    }
    
    // Add RRT_g nodes to path.
    int goalID_g = RRT_g.num - 1;
    while (RRT_g.parent[goalID_g] != goalID_g){
        path.push_back(RRT_g.RRTNode[goalID_g]);
        goalID_g = RRT_g.parent[goalID_g];
    }
    path.push_back(RRT_g.RRTNode[goalID_g]);
    reverse(path.begin(), path.end());

    // Add RRT_s nodes to path.
    int goalID_s = RRT_s.num - 1;
    goalID_s = RRT_s.parent[goalID_s];
    while (RRT_s.parent[goalID_s] != goalID_s){
        path.push_back(RRT_s.RRTNode[goalID_s]);
        goalID_s = RRT_s.parent[goalID_s];
    }
    path.push_back(RRT_s.RRTNode[goalID_s]);
    num_pathpts = path.size();
    numofsamples = num_pathpts;

    // Update the plan variable.
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    for (int i = 0; i < numofsamples; ++i){
        (*plan)[i] = (double*) malloc(dim*sizeof(double)); 
        vector<double> temp = path[numofsamples - 1 - i];
        for (int j = 0; j < dim; ++j){
            (*plan)[i][j] = temp[j];
        }
    }


}