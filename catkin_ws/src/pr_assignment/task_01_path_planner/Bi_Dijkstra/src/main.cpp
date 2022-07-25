#include <Bi_Dijkstra.cpp>

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
        Bi_Dijkstra biDijkstra = Bi_Dijkstra();
        
        // Set the Map 2D Array
        biDijkstra.uploadMap(map2D);        
        // Set the Start Pose(x, y)
        biDijkstra.setStartNode(0, 0);
        // Set the Goal Pose(x, y)
        biDijkstra.setGoalNode(15, 15);
        // Set Path Visualization to true
        biDijkstra.setVisualization(true);

        // Vector to store the planned path 
        vector<Bi_Dijkstra::Node*> path;
        // Start to Plan
        biDijkstra.startPlan();
        // Retrieve the Path
        biDijkstra.getPlannedPath(path);
    
        // Print the Map with planned path in terminal
        biDijkstra.printMap(path);
}