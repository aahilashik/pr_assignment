#include <Bi_A_Star.h>

#include <chrono>
#include <thread>

#include <algorithm>
#include <cmath>


using namespace std;


template <size_t width, size_t height>
void Bi_A_Star::uploadMap(int (&map2D)[width][height]) {
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
            nodes[x*mapWidth + y].nodeState["isStartExplored"] = false;
            nodes[x*mapWidth + y].nodeState["isGoalExplored"] = false;

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

void Bi_A_Star::setStartNode(int startX, int startY) {
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

void Bi_A_Star::setGoalNode(int goalX, int goalY) {
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

void Bi_A_Star::setVisualization(bool enable) {
    visualize = enable;
}

bool Bi_A_Star::startPlan() {
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

void Bi_A_Star::getPlannedPath(vector<Node*>& path) {
    path.assign(plannedPath.begin(), plannedPath.end());
    return ;    
}

bool Bi_A_Star::setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal) {
    // Cost Lambdas Functions
    auto distance = [](Node* a, Node* b) {
        return sqrtf(pow((a->x - b->x), 2) + pow((a->y - b->y), 2));
    };
    auto heuristic = [distance](Node* a, Node* b) {
        return distance(a, b);
    };

    // Nodes Initialization
    for (int x = 0; x < mapWidth; x++)
        for (int y = 0; y < mapHeight; y++) {
            // Use costMap member variable of map<string, double> type
            // to store cost parameters like path cost, state cost, etc.
            nodes[x * mapWidth + y].costMap["StartPathCost"]  = INFINITY;
            nodes[x * mapWidth + y].costMap["StartTotalCost"] = INFINITY;
            nodes[x * mapWidth + y].costMap["GoalPathCost"]   = INFINITY;
            nodes[x * mapWidth + y].costMap["GoalTotalCost"]  = INFINITY;

            // Use nodeState member variable of map<string, bool> type
            // to store node status conditions like obstacle, free, explored, etc.
            nodes[x * mapWidth + y].nodeState["isStartExplored"] = false;
            nodes[x * mapWidth + y].nodeState["isGoalExplored"]  = false;

            // To set the parent node of each node
            nodes[x * mapWidth + y].parents["StartParent"] = nullptr;
            nodes[x * mapWidth + y].parents["GoalParent"]  = nullptr;
        }

    // Setup starting conditions
    // Set start node as current node
    Node *nodeCurrent = nodeStart;
    // Set Start and Goal Cost of start and goal node as 0.0
    nodeStart->costMap["StartTotalCost"]    = heuristic(nodeStart, nodeGoal);
    nodeStart->costMap["StartPathCost"]     = 0.0f;
    nodeGoal->costMap["GoalTotalCost"]      = heuristic(nodeGoal, nodeStart);
    nodeGoal->costMap["GoalPathCost"]       = 0.0f;    

    openNodesList[0].clear();
    openNodesList[1].clear();
    openNodesList[0].push_back(nodeStart);
    openNodesList[1].push_back(nodeGoal);

    int iteration = 0;
    while (!openNodesList[0].empty() && !openNodesList[1].empty()) {
        iteration++;

        // Check whether the current is intersecting with both direction
        if (nodeCurrent->nodeState["isStartExplored"] && nodeCurrent->nodeState["isGoalExplored"]) {
            nodeMiddle = nodeCurrent;
            break;
        }
        
        // Iterate the process two times for Start and Goal Direction
        // dir = 0 -> Search from Start Pose 
        // dir = 1 -> Search from Goal Pose 
        for (int dir=0; dir<2; dir++) {
            // Sort the open list in ascending order with respect to total cost
            openNodesList[dir].sort([&dir](Node* lhs, Node* rhs){ 
                double lhsCost = lhs->costMap[(dir==0?"StartTotalCost":"GoalTotalCost")]; 
                double rhsCost = rhs->costMap[(dir==0?"StartTotalCost":"GoalTotalCost")];
                return lhsCost < rhsCost; 
            } );
            
            while(!openNodesList[dir].empty() && openNodesList[dir].front()->nodeState[(dir==0?"isStartExplored":"isGoalExplored")])
                openNodesList[dir].pop_front();

            if (openNodesList[dir].empty())
                break;

            nodeCurrent = openNodesList[dir].front();
            nodeCurrent->nodeState[(dir==0?"isStartExplored":"isGoalExplored")] = true; // We only explore a node once

            for (auto nodeNeighbour : nodeCurrent->nodeNeighbours) {
                if (!nodeNeighbour->nodeState[(dir==0?"isStartExplored":"isGoalExplored")] && !nodeNeighbour->nodeState["isObstacle"]) 
                    openNodesList[dir].push_back(nodeNeighbour);

                float minCost = nodeCurrent->costMap[(dir==0?"StartPathCost":"GoalPathCost")] + distance(nodeCurrent, nodeNeighbour);

                if (minCost < nodeNeighbour->costMap[(dir==0?"StartPathCost":"GoalPathCost")]) {
                    nodeNeighbour->parents[(dir==0?"StartParent":"GoalParent")] = nodeCurrent;
                    nodeNeighbour->costMap[(dir==0?"StartPathCost":"GoalPathCost")] = minCost;
                    nodeNeighbour->costMap[(dir==0?"StartTotalCost":"GoalTotalCost")] = nodeNeighbour->costMap[(dir==0?"StartPathCost":"GoalPathCost")] + heuristic(nodeNeighbour, (dir==0?nodeGoal:nodeStart));

                }
            }	
        }

        // Animation
        if (visualize) {
            // Clear the terminal to give video-like output
            system("clear");
            cout << "\n  Iteration : " << iteration << endl << endl;
            printMap();
            this_thread::sleep_for(chrono::milliseconds(100));
        }    
    }

    // Append the Plannned by backtrack the parent over child node
    if (nodeMiddle != nullptr) {
        cout << "INFO : Goal Reached at Iteration " << iteration << "!" << endl;
        plannedPath.push_back(nodeMiddle);
        Node *nodeSP = nodeMiddle->parents["StartParent"];
        Node *nodeGP = nodeMiddle->parents["GoalParent"];
        while (nodeSP != nullptr) {
                plannedPath.push_front(nodeSP);
                // Backtrack the parent from start
                nodeSP = nodeSP->parents["StartParent"];
            }
        while (nodeGP != nullptr) {
                plannedPath.push_back(nodeGP);
                // Backtrack the parent from goal
                nodeGP = nodeGP->parents["GoalParent"];;
            }

    } else return false; 

    return true;
}

void Bi_A_Star::printMap(vector<Node*> path) {
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
                else if (&nodes[x * mapWidth + y] == nodeMiddle)
                    cout << "M ";
                else if (nodes[x * mapWidth + y].nodeState["isObstacle"]) 
                    cout << "8 ";
                else if (count(path.begin(), path.end(), &nodes[x * mapWidth + y]))
                    cout << "+ ";
                else if (nodes[x * mapWidth + y].nodeState["isStartExplored"]) 
                    cout << "` ";
                else if (nodes[x * mapWidth + y].nodeState["isGoalExplored"]) 
                    cout << "' ";
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
    cout << " '`' : Explored FW" << endl;
    cout << " ''' : Explored BW" << endl;
    cout << " 'S' : Start Pose" << endl;
    cout << " 'G' : Goal Pose" << endl;
    cout << " '+' : Planned Path" << endl;

}

void Bi_A_Star::resetNodes() {

    cout << "INFO : Nodes are resetted and Path is cleared!" << endl;

    // Assign Default Initial Values
    for (int x=0; x<mapWidth; x++)
        for (int y=0; y<mapHeight; y++) {
            nodes[x*mapWidth + y].nodeState["isStartExplored"] = false;
            nodes[x*mapWidth + y].nodeState["isGoalExplored"]  = false;
        }

    plannedPath.clear();
}