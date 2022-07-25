#include <Dijkstra.cpp>

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
        Dijkstra dijkstra = Dijkstra();
        
        // Set the Map 2D Array
        dijkstra.uploadMap(map2D);        
        // Set the Start Pose(x, y)
        dijkstra.setStartNode(0, 0);
        // Set the Goal Pose(x, y)
        dijkstra.setGoalNode(15, 15);
        // Set Path Visualization to true
        dijkstra.setVisualization(true);

        // Vector to store the planned path 
        vector<Dijkstra::Node*> path;
        // Start to Plan
        dijkstra.startPlan();
        // Retrieve the Path
        dijkstra.getPlannedPath(path);
    
        // Print the Map with planned path in terminal
        dijkstra.printMap(path);
}