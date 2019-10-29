#include <iostream>
#include <vector>
using namespace std;

/*
Node class is the tree node of RRT, RRT*, RRT connect and PRM.
*/
class Node{
    public:
    vector<double> states; // Represent location in configuration space.


};


class RRT{
    public:
    vector<Node> RRTNode;  // Contains every node of the RRT.

};