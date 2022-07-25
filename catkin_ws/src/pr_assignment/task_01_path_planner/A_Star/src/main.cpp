#include <A_Star.cpp>

// Map Dimension
const int Width      = 16;
const int Height     = 16;

// 2D Map Array ( 1: Valid | 0: Invalid/Obstacle ) 
int map2D[Width][Height] =  {   {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                {1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1},
                                {1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1},
                                {1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1},
                                {1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1},
                                {1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1},
                                {1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
                                {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                {1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                {1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
                                {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1},
                                {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1},
                                {1, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1},
                                {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
                                {1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1}, };

int main() {
        // Initialize the Class Object
        A_Star aStar = A_Star();
        
        // Set the Map 2D Array
        aStar.uploadMap(map2D);        
        // Set the Start Pose(x, y)
        aStar.setStartNode(0, 0);
        // Set the Goal Pose(x, y)
        aStar.setGoalNode(15, 15);
        // Set Path Visualization to true
        aStar.setVisualization(true);

        // Vector to store the planned path 
        vector<A_Star::Node*> path;
        // Start to Plan
        aStar.startPlan();
        // Retrieve the Path
        aStar.getPlannedPath(path);
    
        // Print the Map with planned path in terminal
        aStar.printMap(path);
}