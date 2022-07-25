#include <Dijkstra.h>

#include <chrono>
#include <thread>

#include <algorithm>
#include <cmath>


using namespace std;


template <size_t width, size_t height>
void Dijkstra::uploadMap(int (&map2D)[width][height]) {
    mapWidth  = width;
    mapHeight = height;

    nodes = new Node[width * height];

    cout << "INFO : 2D Map of " << mapWidth << " x " << mapHeight << endl;

    // Assign Default Initial Values
    for (int x=0; x<mapWidth; x++)
        for (int y=0; y<mapHeight; y++) {
            nodes[x*mapWidth + y].x = x;
            nodes[x*mapWidth + y].y = y;
            nodes[x*mapWidth + y].nodeState["isObstacle"] = !map2D[x][y];
            nodes[x*mapWidth + y].nodeState["isExplored"] = false;

            for (int _x=-1; _x<=+1; _x++) {
                for (int _y=-1; _y<=+1; _y++) {
                    // Current/Search Pose
                    if ((_x==0) & (_y==0)) 
                        continue;                              
                    // Wall
                    else if ((_x+x<0) | (_x+x>=mapWidth) | (_y+y<0) | (_y+y>=mapHeight))
                        continue;
                    // Possible Successor
                    else 
                        nodes[x*mapWidth + y].nodeNeighbours.push_back(&nodes[(x + _x) * mapWidth + (y + _y)]);
                }
            }

        }
}

void Dijkstra::setStartNode(int startX, int startY) {
    // If map is empty/null 
    if (nodes == nullptr) {
        cout << "WARN : Unknown Environment! Kindly update the map.." << endl;
        return;
    } 

    // Setup the Start Node
    nodeStart = &(nodes)[startX * mapWidth + startY];

    // If provided Start Node is obstacle
    if (nodeStart->nodeState["isObstacle"]==true) {
        cout << "WARN : Given Start Pose is Invalid/Obstacle!" << endl;
        return;
    } else 
        cout << "INFO : Start Position set successfully!" << endl;
}

void Dijkstra::setGoalNode(int goalX, int goalY) {
    // If map is empty/null 
    if (nodes == nullptr) {
        cout << "WARN : Unknown Environment! Kindly update the map.." << endl;
        return;
    }

    // Setup the Goal Node
    nodeGoal = &(nodes)[goalX * mapWidth + goalY];

    // If provided Goal Node is obstacle
    if ((nodeGoal->nodeState["isObstacle"]==true) || (goalX*mapWidth + goalY >= mapWidth*mapHeight)) {
        cout << "WARN : Given Goal Pose is Invalid/Obstacle!" << endl;
        return;
    } else 
        cout << "INFO : Goal Position set successfully!" << endl;
}

void Dijkstra::setVisualization(bool enable) {
    visualize = enable;
}

bool Dijkstra::startPlan() {
    // Clear the Nodes to explore
    resetNodes();

    auto _begin = chrono::high_resolution_clock::now();

    if (nodes == nullptr)
        throw runtime_error("WARN : Unknown Environment! Kindly update the map...!");

    if (nodeStart == nullptr)
        throw runtime_error("WARN : Unknown Initial Pose! Kindly set the start location...!");
        
    if (nodeGoal == nullptr)
        throw runtime_error("WARN : Unknown Goal Pose! Kindly set the goal location...!");

    if (!setPlannedPath(nodes, nodeStart, nodeGoal))
        return false;

    auto _end = chrono::high_resolution_clock::now();
    chrono::duration<double> _elapsed = _end - _begin;
    cout << "INFO : Elapsed Time : " << _elapsed.count() << " seconds!" << endl;

    return true;
}

void Dijkstra::getPlannedPath(vector<Node*>& path) {
    path.assign(plannedPath.begin(), plannedPath.end());
    return ;    
}

bool Dijkstra::setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal) {
    // Cost Lambdas Functions
    auto distance = [](Node* a, Node* b) {
        return sqrtf(pow((a->x - b->x), 2) + pow((a->y - b->y), 2));
    };

    // Nodes Initialization
    for (int x = 0; x < mapWidth; x++)
        for (int y = 0; y < mapHeight; y++) {
            // Use costMap member variable of map<string, double> type
            // to store cost parameters like path cost, state cost, etc.
            nodes[x * mapWidth + y].costMap["StepCost"] = INFINITY;

            // Use nodeState member variable of map<string, bool> type
            // to store node status conditions like obstacle, free, explored, etc.
            nodes[x * mapWidth + y].nodeState["isExplored"] = false;

            // To set the parent node of each node
            nodes[x * mapWidth + y].parents["Parent"] = nullptr;	// No parents
        }

    // Setup starting conditions
    // Set start node as current node
    Node *nodeCurrent = nodeStart;
    // Set Step Cost of current node as 0.0
    nodeCurrent->costMap["StepCost"] = 0.0f;

    openNodesList.clear();
    openNodesList.push_back(nodeStart);

    int iteration = 0;
    while (!openNodesList.empty() && nodeCurrent != nodeGoal) {
        iteration++;
        openNodesList.sort([](Node* lhs, Node* rhs){ return lhs->costMap["StepCost"] < rhs->costMap["StepCost"]; } );
        
        while(!openNodesList.empty() && openNodesList.front()->nodeState["isExplored"])
            openNodesList.pop_front();

        if (openNodesList.empty())
            break;

        nodeCurrent = openNodesList.front();
        nodeCurrent->nodeState["isExplored"] = true; // We only explore a node once
        
        for (auto nodeNeighbour : nodeCurrent->nodeNeighbours) {
            if (!nodeNeighbour->nodeState["isExplored"] && !nodeNeighbour->nodeState["isObstacle"])
                openNodesList.push_back(nodeNeighbour);

            float minCost = nodeCurrent->costMap["StepCost"] + distance(nodeCurrent, nodeNeighbour);

            if (minCost < nodeNeighbour->costMap["StepCost"]) {
                nodeNeighbour->parents["Parent"] = nodeCurrent;
                nodeNeighbour->costMap["StepCost"] = minCost;
            }
        }

        // Animation
        if (visualize) {
            // Clear the terminal to give video-like output
            system("clear");
            cout << "  Iteration : "<< iteration << endl << endl;
            printMap();
            this_thread::sleep_for(chrono::milliseconds(100));
        }    
    }

    // Append the Plannned by backtrack the parent over child node
    if (nodeCurrent == nodeGoal) {
        cout << "INFO : Goal Reached at Iteration " << iteration << "!" << endl;
        Node *nodePath = nodeGoal;
        while (nodePath->parents["Parent"] != nullptr) {
            plannedPath.push_back(nodePath);
            // Backtrack the parent
            nodePath = nodePath->parents["Parent"];
        }
    } else return false;

    return true;
}

void Dijkstra::printMap(vector<Node*> path) {
    // Print the Map
    for (int y=0; y<mapHeight; y++)
        cout << "--";
    cout << "---\n";
    for (int x=0; x<mapWidth; x++) {
        cout << "| ";
        for (int y=0; y<mapHeight; y++) {
                if (&nodes[x * mapWidth + y] == nodeStart)
                    cout << "S ";
                else if (&nodes[x * mapWidth + y] == nodeGoal)
                    cout << "G ";
                else if (nodes[x * mapWidth + y].nodeState["isObstacle"]) 
                    cout << "8 ";
                else if (count(path.begin(), path.end(), &nodes[x * mapWidth + y]))
                    cout << "+ ";
                else if (nodes[x * mapWidth + y].nodeState["isExplored"]) 
                    cout << "` ";
                else
                    cout << "  ";
        }
        cout << "|" << endl;
    }
    for (int y=0; y<mapHeight; y++)
        cout << "--";
    cout << "---\n";

    // Plot Legend
    cout << " ' ' : Valid Space" << endl;
    cout << " '8' : Invalid Space / Obstacle" << endl;
    cout << " '`' : Explored" << endl;
    cout << " 'S' : Start Pose" << endl;
    cout << " 'G' : Goal Pose" << endl;
    cout << " '+' : Planned Path" << endl;

}

void Dijkstra::resetNodes() {

    cout << "INFO : Nodes are resetted and Path is cleared!" << endl;

    // Assign Default Initial Values
    for (int x=0; x<mapWidth; x++)
        for (int y=0; y<mapHeight; y++)
            nodes[x*mapWidth + y].nodeState["isExplored"] = false;

    plannedPath.clear();
}