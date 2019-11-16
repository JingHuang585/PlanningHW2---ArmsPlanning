To run this RRT and RRT connect planner. Following the example steps in matlab below:

```
1. mex planner.cpp MyRRT.cpp RRT_Connect.cpp
2. startQ = [pi/2 pi/4 pi/2 pi/4 pi/2];
3. goalQ = [pi/8 3*pi/4 pi 0.9*pi 1.5*pi];
4. planner_id = 0;
5. runtest('map1.txt',startQ, goalQ, planner_id);

```