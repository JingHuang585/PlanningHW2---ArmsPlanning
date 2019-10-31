#include "MyRRT.h"

double GETDISTANCE(vector<double>& n1, vector<double>& n2){
    double result = 0;
    for (int i = 0; i < n1.size(); ++i){
        result += pow(n1[i]-n2[i], 2);
    }
    result = sqrt(result);
    return result;
}

void MyRRT::set_epsilon(int val){
    epsilon = val;
}

vector<double> MyRRT::samplePts(){
    /*
    Return:
        New sampled node point.
    */
    double p = (double)rand() / RAND_MAX;
    vector<double> pts (dim, 0);
    if (p < 0.1){
        pts = goal;
    }
    else{
        for (int i = 0; i < dim; ++i){
            double val = (double)rand() / RAND_MAX * (2*M_PI);
            pts[i] = val;
        }
    }
    return pts;
}

vector<double> MyRRT::getNearest(vector<double>& n1, int& nearest){
    /*
    Args:
        n1: New nodes that needs to find the nearest node in the RRT.
        nearest: ID of the nearest node in the RRT.
    Return:
        Nearest node in the RRT.
    */
    vector<double> result;
    double dist = double(INT_MAX);
    for(int i = 0; i < RRTNode.size(); ++i){
        double temp_dist = GETDISTANCE(n1, RRTNode[i]);
        if (temp_dist < dist){
            dist = temp_dist;
            nearest = i;
        }
    }
    result = RRTNode[nearest];
    //parent[RRTNode.size()] = nearest;
    return result;
}

vector<double> MyRRT::getSampled(vector<double>& n1, vector<double>& n2){
    /*
    Args:
        n1: Sampled node.
        n2: Nearest RRT node with regard to n1.
    Return:
        Final node to be sampled that has a distance of epsilon with n2.
    */
    vector<double> result (n1.size(), 0);
    double dist = GETDISTANCE(n1, n2);
    double ratio = epsilon / dist;
    for (int i = 0; i < n1.size(); ++i){
        result[i] = n2[i] + (n1[i] - n2[i]) * ratio;
    }
    return result;
}

bool MyRRT::checkCollision(vector<double>& n1, vector<double>& n2){
    /*
    Args:
        n1: Node to be sampled.
        n2: Nearest node in the RRT.
    Return:
        Boolean indicating whether there will be collision from n2 to n1.
    */
    int interpolate_num = 10;                                // Interpolation number between two nodes.
    double dist = GETDISTANCE(n1, n2);
    for (int i = 1; i <= interpolate_num; ++i){
        vector<double> temp (n2.size(), 0);
        double* angles;
        angles = (double*) malloc(dim*sizeof(double)); 
        for (int j = 0; j < n2.size(); ++j){
            angles[j] = n2[j] + dist * i / interpolate_num;
        }
        // Check if there is collision
        bool isCollide = IsValidArmConfiguration(angles, dim, RRTMap, x_size, y_size);
        if (!isCollide){
            return true;
        }
    }
    return false;
}

void MyRRT::planning(double*** plan, int& numofsamples){
    
    bool terminate = false;
    double threshold = 1;
    int goalID = -1;
    while (!terminate){
        vector<double> temp_node = samplePts();                               // Sample a new point.
        int nearestID = -1;
        vector<double> nearest_node = getNearest(temp_node, nearestID);       // Get the nearest RRT node of the temporary sampled node.
        vector<double> sampled_node = getSampled(temp_node, nearest_node);    // Get the sampled RRT node using epsilon.
        bool collision = checkCollision(sampled_node, nearest_node);
        if (!collision){
            RRTNode.push_back(sampled_node);
            ++num;
            parent[num-1] = nearestID;                                        // New point's ID is: num - 1, parent's ID is: nearestID.
            if (GETDISTANCE(sampled_node, goal) < threshold){
                // Reach the goal!
                goalID = num - 1;
                break;
            }
        }
    }
    // Backtrack the goalID to get a path.
    while (parent[goalID] != goalID){
        path.push_back(RRTNode[goalID]);
        goalID = parent[goalID];
    }
    path.push_back(RRTNode[goalID]);
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