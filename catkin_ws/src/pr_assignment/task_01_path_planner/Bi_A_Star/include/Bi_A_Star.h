#include <iostream>

#include <vector>
#include <list>
#include <map>

using namespace std;

class Bi_A_Star {
    public:
        // Node Structure
        struct Node {
            // Coordinates/Pose of the Node
            int x, y;

            // To store Cost and Pose Related Parameter
            map<string, double> costMap;
            map<string, bool>   nodeState;

            // Vector/List of Neighbours
            vector<Node*>   nodeNeighbours;

            // Parent Node of each node
            map<string, Node*> parents;
        };

        // Planned Path List  
        list<Node*> plannedPath;

        // Map Dimensions
        int mapWidth    = 0;
        int mapHeight   = 0;

    private:
        // Nodes Array
        Node *nodes = nullptr;
        
        // Start & Goal Nodes
        Node *nodeStart = nullptr;
        Node *nodeGoal = nullptr;
        // Meeting Node since Bidirectional
        Node *nodeMiddle = nullptr;

        // Data Members
        list<Node*> openNodesList[2];

        // Visualization Flag
        bool visualize = false;
        
    public:
        /* -------- Constructors -------- */
        Bi_A_Star() {
        }

        /* -------- Member Functions -------- */
        // Function with 2D Map as input
        template <size_t width, size_t height>
        void uploadMap(int (&map2D)[width][height]);

        void setStartNode(int x, int y);
        void setGoalNode(int x, int y);

        void setVisualization(bool enable);

        bool startPlan();
        
        void getPlannedPath(vector<Node*>& path);
        bool setPlannedPath(Node *nodes, Node *nodeStart, Node *nodeGoal);

        void resetNodes();
        void printMap(vector<Node*> path={});
};

