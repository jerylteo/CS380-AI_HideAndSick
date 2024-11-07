#pragma once
#include "Misc/PathfindingDetails.hpp"

class AStarPather
{
public:
    /* 
        The class should be default constructible, so you may need to define a constructor.
        If needed, you can modify the framework where the class is constructed in the
        initialize functions of ProjectTwo and ProjectThree.
    */

    /* ************************************************** */
    // DO NOT MODIFY THESE SIGNATURES
    bool initialize();
    void shutdown();
    PathResult compute_path(PathRequest &request);
    /* ************************************************** */

    /*
        You should create whatever functions, variables, or classes you need.
        It doesn't all need to be in this header and cpp, structure it whatever way
        makes sense to you.
    */

    enum class OnList : uint8_t
    {
        NONE,
        OPEN,
        CLOSED
    };

    struct Node
    {
        Node* parent;  
        GridPos gridPos;        
        float finalCost;         // f = g + h (total estimated cost)
        float givenCost;         // g (cost from the start node to this node)
        OnList onList;      
    };

    float AStarPather::calculateHeuristic(const GridPos& from, const GridPos& to, Heuristic heuristic, float weight) const;


private:
    int MAX_MAP_SIZE = 40;     // Maximum map dimensions
    static const int MAX_MAPS = 7;

    std::vector<std::vector<std::vector<bool>>> precomputedWalls;

    std::vector<std::vector<std::vector<float>>> floydWarshallDistances;
    bool hasPrecomputedFloydWarshall = false;
    void precomputeFloydWarshall();


    const std::array<GridPos, 8> NEIGHBOR_OFFSETS = {
        GridPos(-1, 0), GridPos(1, 0), GridPos(0, -1), GridPos(0, 1),
        GridPos(-1, -1), GridPos(-1, 1), GridPos(1, -1), GridPos(1, 1)
    };

    Node nodes[40][40];
    std::vector<Node*> openList;

    void precomputeWallsForCurrentMap();
    void resizeMap();
    Node* getCheapestNode();
    std::vector<GridPos> getNeighbors(const Node* node) const;
    bool isValidPosition(const GridPos& pos) const;
    float getMovementCost(const GridPos& from, const GridPos& to) const;
    void resetNodes();

};
