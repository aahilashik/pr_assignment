#include <Bi_A_Star.cpp>

// Map Dimension
const int Width      = 16;
const int Height     = 16;

// 2D Map Array ( 1: Valid | 0: Obstacle ) 
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
        Bi_A_Star biAStar = Bi_A_Star();
        
        // Set the Map 2D Array
        biAStar.uploadMap(map2D);        
        // Set the Start Pose(x, y)
        biAStar.setStartNode(0, 0);
        // Set the Goal Pose(x, y)
        biAStar.setGoalNode(15, 15);
        // Set Path Visualization to true
        biAStar.setVisualization(true);

        // Vector to store the planned path 
        vector<Bi_A_Star::Node*> path;
        // Start to Plan
        biAStar.startPlan();
        // Retrieve the Path
        biAStar.getPlannedPath(path);
    
        // Print the Map with planned path in terminal
        biAStar.printMap(path);
}