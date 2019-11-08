#include "MyRRT.h"


double GETDISTANCE(vector<double>& n1, vector<double>& n2){
    double result = 0;
    for (int i = 0; i < n1.size(); ++i){
        result += pow(n1[i]-n2[i], 2);
    }
    result = sqrt(result);
    return result;
}

void MyRRT::set_epsilon(double val){
    epsilon = val;
}

void MyRRT::set_threshold(double val){
    threshold = val;
}

void MyRRT::set_maxnum(int val){
    max_num = val;
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

vector<double> MyRRT::getNearest(vector<double>& q_rand, int& nearest){
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
        double temp_dist = GETDISTANCE(q_rand, RRTNode[i]);
        if (temp_dist < dist){
            dist = temp_dist;
            nearest = i;
        }
    }
    result = RRTNode[nearest];
    return result;
}

vector<double> MyRRT::getSampled(vector<double>& q_rand, vector<double>& q_near){
    /*
    Args:
        q_rand: Sampled node.
        q_near: Nearest RRT node with regard to n1.
    Return:
        Final node to be sampled that has a distance of epsilon with n2.
    */
    vector<double> result (q_rand.size(), 0);
    double dist = GETDISTANCE(q_rand, q_near);
    if (dist <= epsilon){
        result = q_rand;
    }
    else{
        double ratio = epsilon / dist;
        for (int i = 0; i < q_rand.size(); ++i){
            result[i] = q_near[i] + (q_rand[i] - q_near[i]) * ratio;
        }
    }
    return result;
}

bool MyRRT::checkCollision(vector<double>& q_samp, vector<double>& q_near){
    /*
    Args:
        q_samp: Node to be sampled.
        q_near: Nearest node in the RRT.
    Return:
        Boolean indicating whether there will be collision from q_near to q_samp.
    */
    int interpolate_num = 10;                                // Interpolation number between two nodes.
    //double dist = GETDISTANCE(q_samp, q_near);
    for (int i = 1; i <= interpolate_num; ++i){
        vector<double> temp (q_near.size(), 0);
        double* angles;
        angles = (double*) malloc(dim*sizeof(double)); 
        for (int j = 0; j < q_near.size(); ++j){
            angles[j] = q_near[j] + (q_samp[j] - q_near[j]) * i / interpolate_num;
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
    /*
    Args:
        plan: Planned path. A pointer points to a 2D matrix, which represents the planned path.
        numofsamples: Number of points along the planned path.
    */
    bool terminate = false;
    int goalID = 0;
    int count = 0;
    while (!terminate){
        ++count;
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
                terminate = true;
                break;
            }
        }
        // Currently don't use count to terminate.
        if (count >= max_num){
            terminate = true;
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