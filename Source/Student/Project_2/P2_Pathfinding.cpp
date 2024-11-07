#include <pch.h>
#include "Projects/ProjectTwo.h"
#include "P2_Pathfinding.h"

constexpr float SQRT_2 = 1.41421356237;

#pragma region Extra Credit
bool ProjectTwo::implemented_floyd_warshall()
{
    return true;
}

bool ProjectTwo::implemented_goal_bounding()
{
    return false;
}

bool ProjectTwo::implemented_jps_plus()
{
    return false;
}
#pragma endregion

bool AStarPather::initialize()
{
    Callback cb = std::bind(&AStarPather::resizeMap, this);
    Messenger::listen_for_message(Messages::MAP_CHANGE, cb);

    for (int row = 0; row < MAX_MAP_SIZE; ++row)
    {
        for (int col = 0; col < MAX_MAP_SIZE; ++col)
        {
            nodes[row][col].gridPos = { row, col };
            nodes[row][col].parent = nullptr;
            nodes[row][col].finalCost = 0.0f;
            nodes[row][col].givenCost = 0.0f;
            nodes[row][col].onList = OnList::NONE;
        }
    }
    
    precomputedWalls.reserve(MAX_MAPS);
    precomputedWalls.resize(MAX_MAPS);

    floydWarshallDistances.reserve(MAX_MAPS);
    floydWarshallDistances.resize(MAX_MAPS);



    for (int mapIndex = 0; mapIndex < MAX_MAPS; ++mapIndex) {
        terrain->goto_next_map();
        precomputeWallsForCurrentMap();

        if (ProjectTwo::implemented_floyd_warshall()) {
            floydWarshallDistances[mapIndex].resize(terrain->get_map_height(), std::vector<float>(terrain->get_map_height(), std::numeric_limits<float>::infinity()));
			precomputeFloydWarshall();
		}
    }

    if (ProjectTwo::implemented_floyd_warshall()) {
		std::cout << "[0][0]: " << floydWarshallDistances[6][0][0] << std::endl;
		std::cout << "[0][1]: " << floydWarshallDistances[6][0][1] << std::endl;
		std::cout << "[0][39]: " << floydWarshallDistances[6][0][39] << std::endl;
	}


    return true; // return false if any errors actually occur, to stop engine initialization
}

void AStarPather::shutdown()
{
    /*
        Free any dynamically allocated memory or any other general house-
        keeping you need to do during shutdown.
    */
}

void AStarPather::precomputeWallsForCurrentMap() {
    int currentMapIndex = terrain->get_map_index();

    int currentMapSize = terrain->get_map_height();

    precomputedWalls.reserve(currentMapSize);
    precomputedWalls[currentMapIndex].resize(currentMapSize, std::vector<bool>(currentMapSize, false));

    for (int row = 0; row < currentMapSize; ++row) {
        for (int col = 0; col < currentMapSize; ++col) {
            precomputedWalls[currentMapIndex][row][col] = terrain->is_wall({ row, col });
        }
    }
}

void AStarPather::precomputeFloydWarshall() {
    int currentMapIndex = terrain->get_map_index();
    int currentMapSize = terrain->get_map_height();

    floydWarshallDistances[currentMapIndex].clear();
    floydWarshallDistances[currentMapIndex].resize(currentMapSize, std::vector<float>(currentMapSize, std::numeric_limits<float>::infinity()));

    for (int i = 0; i < currentMapSize; ++i) {
        for (int j = 0; j < currentMapSize; ++j) {
            floydWarshallDistances[currentMapIndex][i][j] = (i == j) ? 0.0f : std::numeric_limits<float>::infinity();
        }
    }

    // 2. Initial Distance Setup (Calculating neighbors and costs within the function)
    for (int row = 0; row < currentMapSize; ++row) {
        for (int col = 0; col < currentMapSize; ++col) {
            // Calculate neighbors and movement costs directly:
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    if (dx == 0 && dy == 0) continue;  // Skip the current cell

                    int neighborRow = row + dx;
                    int neighborCol = col + dy;

                    if (isValidPosition({ neighborRow, neighborCol }) && !terrain->is_wall({ neighborRow, neighborCol })) {
                        // Calculate cost (1 for horizontal/vertical, sqrt(2) for diagonal)
                        float cost = (std::abs(dx) == 1 && std::abs(dy) == 1) ? SQRT_2 : 1.0f;
                        floydWarshallDistances[currentMapIndex][row][col] = std::min(
                            floydWarshallDistances[currentMapIndex][row][col],
                            cost
                        );
                    }
                }
            }

            if (currentMapIndex == 6) {
                std::cout << "floydWarshallDistances[" << row << "][" << col << "]: " << floydWarshallDistances[currentMapIndex][row][col] << std::endl;
            }
        }
    }



    for (int k = 0; k < currentMapSize; ++k) {
        for (int i = 0; i < currentMapSize; ++i) {
            for (int j = 0; j < currentMapSize; ++j) {
                // Check for underflow (important when dealing with infinity)
                if (floydWarshallDistances[currentMapIndex][i][k] != std::numeric_limits<float>::infinity() &&
                    floydWarshallDistances[currentMapIndex][k][j] != std::numeric_limits<float>::infinity()) {
                    floydWarshallDistances[currentMapIndex][i][j] = std::min(
                        floydWarshallDistances[currentMapIndex][i][j],
                        floydWarshallDistances[currentMapIndex][i][k] + floydWarshallDistances[currentMapIndex][k][j]
                    );
                }
            }
        }
    }



    hasPrecomputedFloydWarshall = true;
}



void AStarPather::resizeMap() {
    MAX_MAP_SIZE = terrain->get_map_height();
    //openList.reserve(MAX_MAP_SIZE * MAX_MAP_SIZE);
}

PathResult AStarPather::compute_path(PathRequest &request)
{

    if (request.newRequest)
    {
        resetNodes();
        openList.clear();

        GridPos startPos = terrain->get_grid_position(request.start);
        GridPos goalPos = terrain->get_grid_position(request.goal);

        // Check if start or goal positions are valid
        if (!isValidPosition(startPos) || !isValidPosition(goalPos))
        {
            return PathResult::IMPOSSIBLE; 
        }

        Node& startNode = nodes[startPos.row][startPos.col];
        startNode.parent = nullptr;
        startNode.givenCost = 0.0f;
        startNode.finalCost = calculateHeuristic(startPos, goalPos, request.settings.heuristic, request.settings.weight);
        startNode.onList = OnList::OPEN;
        //addToOpenList(&startNode);
        openList.push_back(&startNode);
    }
    
    while (!openList.empty())
    {
        Node* current = getCheapestNode();

        if (current->gridPos == terrain->get_grid_position(request.goal))
        {
            request.path.clear();
            while (current != nullptr)
            {
                request.path.push_front(terrain->get_world_position(current->gridPos));
                current = current->parent;
            }

 
            if (request.settings.rubberBanding && request.path.size() > 3) {
                auto it = request.path.begin();
                while (it != request.path.end() && std::next(it, 2) != request.path.end()) {
                    auto nextNextIt = std::next(it, 2);

                    GridPos startPos = terrain->get_grid_position(*it);
                    GridPos endPos = terrain->get_grid_position(*nextNextIt);

                    bool wallFound = false;
                    for (int r = std::min(startPos.row, endPos.row); r <= std::max(startPos.row, endPos.row); ++r) {
                        for (int c = std::min(startPos.col, endPos.col); c <= std::max(startPos.col, endPos.col); ++c) {
                            if (terrain->is_wall({ r, c })) {
                                wallFound = true;
                                break;
                            }
                        }
                        if (wallFound) break;
                    }

                    if (!wallFound) {
                        it = request.path.erase(std::next(it));
                        it--;
                    }
                    else {
						++it;
					}
                }
            }

            if (request.settings.rubberBanding && request.settings.smoothing && request.path.size() > 3) {
                auto it = request.path.begin();
                while (it != request.path.end() && std::next(it) != request.path.end()) {
                    Vec3 current = *it;
                    Vec3 next = *std::next(it);
                    float distance = (next - current).Length();

                    float width = terrain->get_world_position({ 1, 0 }).x - terrain->get_world_position({ 0, 0 }).x;

                    if (distance > 1.5f * width) {
						int numPointsToAdd = static_cast<int>(distance / (1.5f * width));
						Vec3 step = (next - current) / static_cast<float>((numPointsToAdd + 1));

						for (int i = 0; i < numPointsToAdd; ++i) {
							it = request.path.insert(std::next(it), current + static_cast<float>((i + 1)) * step);
						}
					}
                    ++it;
                }
            }

            if (request.settings.smoothing && request.path.size() >= 4) {
                WaypointList smoothedPath;
                smoothedPath.push_back(request.path.front());

                for (auto it = std::next(request.path.begin());
                    std::next(it) != request.path.end();
                    ++it) {
                    Vec3 p0 = *std::prev(it);
                    Vec3 p1 = *it;
                    Vec3 p2 = *std::next(it);
                    Vec3 p3 = *std::next(std::next(it));                    
                    if (std::next(std::next(it)) == request.path.end()) {
                        p3 = *std::next(it);
                    }

                    for (float t = 0.0f; t < 1.0f; t += 0.25f) {
                        Vec3 point = XMVectorCatmullRom(p0, p1, p2, p3, t);
                        smoothedPath.push_back(point);
                    }
                }
                smoothedPath.push_back(request.path.back());
                request.path = smoothedPath;
            }
            request.newRequest = true;

            return PathResult::COMPLETE;
        }

        current->onList = OnList::CLOSED;
        if (request.settings.debugColoring)
            terrain->set_color(current->gridPos, Colors::Yellow);

        // Process neighbors
        //const NeighborData& currentNeighbors = precomputedNeighbors[terrain->get_map_index()][current->gridPos.row][current->gridPos.col];

        for (const GridPos& neighborPos : getNeighbors(current))
        //for (int i = 0; i < currentNeighbors.numNeighbors; ++i)
        {
            //int neighborIndex = currentNeighbors.neighborIndices[i];
            //GridPos neighborPos = { neighborIndex / MAX_MAP_SIZE, neighborIndex % MAX_MAP_SIZE };

            Node* neighbor = &nodes[neighborPos.row][neighborPos.col];

            float newGivenCost = current->givenCost + getMovementCost(current->gridPos, neighborPos);
            float newFinalCost = newGivenCost + calculateHeuristic(neighborPos, terrain->get_grid_position(request.goal), request.settings.heuristic, request.settings.weight);

            // Check if neighbor is in the closed list and its new cost is lower
            if (neighbor->onList == OnList::CLOSED && newFinalCost < neighbor->finalCost)
            {
                // Re-open the node
                neighbor->finalCost = newFinalCost;
                neighbor->givenCost = newGivenCost;
                neighbor->parent = current;
                neighbor->onList = OnList::OPEN;
                //addToOpenList(neighbor);
                openList.push_back(neighbor);

                if (request.settings.debugColoring)
                    terrain->set_color(neighborPos, Colors::Blue);
            }
            else if (neighbor->onList == OnList::NONE || newGivenCost < neighbor->givenCost)
            {
                neighbor->parent = current;
                neighbor->givenCost = newGivenCost;
                neighbor->finalCost = newFinalCost;
                neighbor->onList = OnList::OPEN;
                //addToOpenList(neighbor);
                openList.push_back(neighbor);

                if (request.settings.debugColoring)
                    terrain->set_color(neighborPos, Colors::Blue);
            }
        }

        if (request.settings.singleStep) {
            request.newRequest = false;
            return PathResult::PROCESSING;
        }
    }

    return PathResult::IMPOSSIBLE;
}

inline float AStarPather::calculateHeuristic(const GridPos& from, const GridPos& to, Heuristic heuristic, float weight) const
{
    if (ProjectTwo::implemented_floyd_warshall() && hasPrecomputedFloydWarshall) {
        int currentMapIndex = terrain->get_map_index();
        return weight * floydWarshallDistances[currentMapIndex][from.row][from.col];
    }
    else {
        int xDiff = std::abs(from.col - to.col);
        int yDiff = std::abs(from.row - to.row);

        switch (heuristic)
        {
        case Heuristic::OCTILE:
            return weight * (std::min(xDiff, yDiff) * SQRT_2 + std::max(xDiff, yDiff) - std::min(xDiff, yDiff));
        case Heuristic::MANHATTAN:
            return weight * (xDiff + yDiff);
        case Heuristic::CHEBYSHEV:
            return weight * std::max(xDiff, yDiff);
        case Heuristic::EUCLIDEAN:
            return weight * std::sqrt(static_cast<float>(xDiff * xDiff + yDiff * yDiff));
        case Heuristic::INCONSISTENT:
            if ((from.row + from.col) % 2 > 0)
            {
                return weight * std::sqrt(static_cast<float>(xDiff * xDiff + yDiff * yDiff)); // Euclidean distance
            }
            else
            {
                return 0.0f;
            }
        default:
            return 0.0f;
        }
    }


}

//void AStarPather::addToOpenList(Node* node)
//{
//    openList.push_back(node);
//}

AStarPather::Node* AStarPather::getCheapestNode()
{
    auto cheapestNodeIter = std::min_element(openList.begin(), openList.end(),
        [](const Node* a, const Node* b) {
            return a->finalCost < b->finalCost ||
                (a->finalCost == b->finalCost && a->givenCost < b->givenCost);
        });

    Node* cheapestNode = *cheapestNodeIter;
    openList.erase(cheapestNodeIter); 
    return cheapestNode;
}

std::vector<GridPos> AStarPather::getNeighbors(const Node* node) const
{
    std::vector<GridPos> neighbors;
    neighbors.reserve(8);

    for (const GridPos& offset : NEIGHBOR_OFFSETS) {
        if (offset.row == 0 && offset.col == 0) continue;

        GridPos neighborPos(node->gridPos.row + offset.row, node->gridPos.col + offset.col);

        int currentMapIndex = terrain->get_map_index();
        if (terrain->is_valid_grid_position(neighborPos) &&
            !precomputedWalls[currentMapIndex][neighborPos.row][neighborPos.col]) {
            if (offset.row != 0 && offset.col != 0) {
                if (precomputedWalls[currentMapIndex][node->gridPos.row][node->gridPos.col + offset.col] ||
                    precomputedWalls[currentMapIndex][node->gridPos.row + offset.row][node->gridPos.col]) {
                    continue;
                }
            }
            neighbors.push_back(neighborPos);
        }
        
    }
    return neighbors;
}

bool AStarPather::isValidPosition(const GridPos& pos) const
{
    return (pos.row >= 0 && pos.row < MAX_MAP_SIZE) &&
        (pos.col >= 0 && pos.col < MAX_MAP_SIZE) &&
        terrain->is_valid_grid_position(pos);
}

inline float AStarPather::getMovementCost(const GridPos& from, const GridPos& to) const
{

    float cost = 1.0f;

    if ((from.row ^ to.row) & (from.col ^ to.col)) { 
        cost *= SQRT_2;
    }

    //if (terrain->is_wall(to)) 
    if (precomputedWalls[terrain->get_map_index()][to.row][to.col])
    {
        return std::numeric_limits<float>::infinity();
    }
    return cost;
}

void AStarPather::resetNodes()
{
    for (int row = 0; row < MAX_MAP_SIZE; ++row)
    {
        for (int col = 0; col < MAX_MAP_SIZE; ++col)
        {
            nodes[row][col].onList = OnList::NONE;
        }
    }

}
