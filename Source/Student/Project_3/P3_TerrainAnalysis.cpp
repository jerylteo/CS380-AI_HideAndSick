#include <pch.h>
#include "Terrain/TerrainAnalysis.h"
#include "Terrain/MapMath.h"
#include "Agent/AStarAgent.h"
#include "Terrain/MapLayer.h"
#include "Projects/ProjectThree.h"

#include <iostream>

bool ProjectThree::implemented_fog_of_war() const // extra credit
{
    return false;
}

float distance_to_closest_wall(int row, int col)
{
    /*
        Check the euclidean distance from the given cell to every other wall cell,
        with cells outside the map bounds treated as walls, and return the smallest
        distance.  Make use of the is_valid_grid_position and is_wall member
        functions in the global terrain to determine if a cell is within map bounds
        and a wall, respectively.
    */

    // WRITE YOUR CODE HERE
    const float mapSize = (float)terrain->get_map_width();
    float minDistance = FLT_MAX;

    for (int i = -1; i < mapSize + 1; i++)
	{
		for (int j = -1; j < mapSize + 1; j++)
		{
            if (i == row && j == col) continue;

			if (!terrain->is_valid_grid_position(i, j) || terrain->is_wall(i, j))
			{
                float di = (float)(i - row);
                float dj = (float)(j - col);
                float distance = sqrtf(di * di + dj * dj); // Calculate Euclidean distance
                minDistance = std::min(minDistance, distance);
                if (minDistance <= 1.f) return minDistance; // Early exit if the distance is less than 1
			}
		}
	}
    
    return minDistance;
}

bool is_clear_path(int row0, int col0, int row1, int col1)
{
    /*
        Two cells (row0, col0) and (row1, col1) are visible to each other if a line
        between their centerpoints doesn't intersect the four boundary lines of every
        wall cell.  You should puff out the four boundary lines by a very tiny amount
        so that a diagonal line passing by the corner will intersect it.  Make use of the
        line_intersect helper function for the intersection test and the is_wall member
        function in the global terrain to determine if a cell is a wall or not.
    */

    // v2
    // WRITE YOUR CODE HERE
    if (terrain->is_wall(row0, col0) || terrain->is_wall(row1, col1)) return false;
    const float mapSize = (float)terrain->get_map_width();
    const float EPSILON = 0.01f;

    Vec2 start((float)row0, (float)col0);
    Vec2 end((float)row1, (float)col1);

    // Bounding box for the line segment, first test for optimisation
    float lineMinX = std::min(start.x, end.x);
    float lineMaxX = std::max(start.x, end.x);
    float lineMinY = std::min(start.y, end.y);
    float lineMaxY = std::max(start.y, end.y);

    for (int row = 0; row < mapSize; row++) {
        for (int col = 0; col < mapSize; col++) {
            if (terrain->is_wall(row, col)) {
                // Bounding box for the wall cell
                float wallMinX = row - 0.5f - EPSILON;
                float wallMaxX = row + 0.5f + EPSILON;
                float wallMinY = col - 0.5f - EPSILON;
                float wallMaxY = col + 0.5f + EPSILON;

                if (lineMaxX >= wallMinX && lineMinX <= wallMaxX &&
                    lineMaxY >= wallMinY && lineMinY <= wallMaxY) {
                    // Wall cell intersects the line segment bounding box, check for intersection
                    Vec2 wallCorners[4] = {
                        Vec2(row - 0.5f - EPSILON, col + 0.5f + EPSILON),  // Top-left
                        Vec2(row + 0.5f + EPSILON, col + 0.5f + EPSILON),  // Top-right
                        Vec2(row + 0.5f + EPSILON, col - 0.5f - EPSILON),  // Bottom-right
                        Vec2(row - 0.5f - EPSILON, col - 0.5f - EPSILON)   // Bottom-left
                    };

                    // Check for intersection with each of the wall's four sides
                    if (line_intersect(start, end, wallCorners[0], wallCorners[1])) return false;
                    else if (line_intersect(start, end, wallCorners[1], wallCorners[2])) return false;
                    else if (line_intersect(start, end, wallCorners[2], wallCorners[3])) return false;
                    else if (line_intersect(start, end, wallCorners[3], wallCorners[0])) return false;
                }


            }
        }
    }

    return true; 
}

void analyze_openness(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the value 1 / (d * d),
        where d is the distance to the closest wall or edge.  Make use of the
        distance_to_closest_wall helper function.  Walls should not be marked.
    */

    // WRITE YOUR CODE HERE
    const float mapSize = (float)terrain->get_map_width();
    for (int row = 0; row < mapSize; row++) {
		for (int col = 0; col < mapSize; col++) {
			if (terrain->is_wall(row, col)) continue;

			float distance = distance_to_closest_wall(row, col);
			layer.set_value(row, col, 1 / (distance * distance));
		}
	}
}

void analyze_visibility(MapLayer<float> &layer)
{
    /*
        Mark every cell in the given layer with the number of cells that
        are visible to it, divided by 160 (a magic number that looks good).  Make sure
        to cap the value at 1.0 as well.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    // v2
    // WRITE YOUR CODE HERE
    const float mapSize = (float)terrain->get_map_width();
    const int visibilityThreshold = 160;

    for (int row = 0; row < mapSize; row++) {
        for (int col = 0; col < mapSize; col++) {
            if (terrain->is_wall(row, col)) continue;

            int visibleCells = 0;
            for (int i = 0; i < mapSize; i++) {
                for (int j = (i == row ? col + 1 : 0) ; j < mapSize; j++) {
                    if (is_clear_path(row, col, i, j)) {
                        visibleCells++;
                        if (visibleCells >= visibilityThreshold) {
                            layer.set_value(row, col, 1.0f);
                            break;
                        }
                    }
                }
                if (visibleCells >= visibilityThreshold) break;
            }
            if (visibleCells < visibilityThreshold)
                layer.set_value(row, col, (float)visibleCells / 160.0f);
        }
    }
}

void analyze_visible_to_cell(MapLayer<float> &layer, int row, int col)
{
    /*
        For every cell in the given layer mark it with 1.0
        if it is visible to the given cell, 0.5 if it isn't visible but is next to a visible cell,
        or 0.0 otherwise.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */

    // WRITE YOUR CODE HERE
    const float mapSize = (float)terrain->get_map_width();
    for (int i = 0; i < mapSize; i++) {
		for (int j = 0; j < mapSize; j++) {
            layer.set_value(i, j, 0.f);

            if (i == row && j == col) 
				layer.set_value(i, j, 1.0f);
			else if (!terrain->is_wall(i, j) && is_clear_path(row, col, i, j))
				layer.set_value(i, j, 1.0f);
		}
	}

    for (int i = 0; i < mapSize; i++) {
        for (int j = 0; j < mapSize; j++) {
            if (layer.get_value(i, j) == 1.0f && (i != row || j != col)) {
                for (int y = std::max(0, i - 1); y <= std::min((int)mapSize - 1, i + 1); y++) {
                    for (int x = std::max(0, j - 1); x <= std::min((int)mapSize - 1, j + 1); x++) {
                        if (layer.get_value(y, x) == .0f && 
                            !terrain->is_wall(y, x) &&
                            is_clear_path(i, j, y ,x)) layer.set_value(y, x, 0.5f);
                    }
                }
            }
        }
    }

}

void analyze_agent_vision(MapLayer<float> &layer, const Agent *agent)
{
    /*
        For every cell in the given layer that is visible to the given agent,
        mark it as 1.0, otherwise don't change the cell's current value.

        You must consider the direction the agent is facing.  All of the agent data is
        in three dimensions, but to simplify you should operate in two dimensions, the XZ plane.

        Take the dot product between the view vector and the vector from the agent to the cell,
        both normalized, and compare the cosines directly instead of taking the arccosine to
        avoid introducing floating-point inaccuracy (larger cosine means smaller angle).

        Give the agent a field of view slighter larger than 180 degrees.

        Two cells are visible to each other if a line between their centerpoints doesn't
        intersect the four boundary lines of every wall cell.  Make use of the is_clear_path
        helper function.
    */
    const float mapSize = (float)terrain->get_map_width();
    GridPos agentGridPos = terrain->get_grid_position(agent->get_position()); // Use get_grid_position
    Vec3 agentDir = agent->get_forward_vector();

    // Project agentDir to XZ plane
    Vec2 agentDir2D(agentDir.x, agentDir.z);
    agentDir2D.Normalize(); // Normalize the direction vector

    // FOV slightly larger than 180 degrees (in radians)
    float fovAngle = (180.0f + 5.f) * M_PI / 180.f;
    float cosFov = cos(fovAngle / 2.0f);

    for (int row = 0; row < mapSize; row++) {
        for (int col = 0; col < mapSize; col++) {
            // Calculate the vector from the agent to the cell (in grid coordinates)
            Vec2 toCell((float)row - agentGridPos.row, (float)col - agentGridPos.col);
            toCell.Normalize();

            // Calculate the dot product
            float dotProduct = agentDir2D.Dot(toCell);

            // Check if the cell is within FOV and if there's a clear path (using grid coordinates)
            if (dotProduct >= cosFov && is_clear_path(agentGridPos.row, agentGridPos.col, row, col)) {
                layer.set_value(row, col, 1.0f); // Mark the cell as visible
            }
        }
    }
}

void propagate_solo_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    /*
        For every cell in the given layer:

            1) Get the value of each neighbor and apply decay factor
            2) Keep the highest value from step 1
            3) Linearly interpolate from the cell's current value to the value from step 2
               with the growing factor as a coefficient.  Make use of the lerp helper function.
            4) Store the value from step 3 in a temporary layer.
               A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */
    // v2
    // WRITE YOUR CODE HERE
    const float mapSize = (float)terrain->get_map_width();
    float tempLayer[40][40];
    float oneMinusGrowth = 1.0f - growth;

    std::array<std::pair<int, int>, 4> neighbors = {{ { -1, 0 }, { 1, 0 }, { 0, -1 }, { 0, 1 } } };
    std::array<std::pair<int, int>, 4> diagonals = {{ { -1, -1 }, { -1, 1 }, { 1, -1 }, { 1, 1 } } };
    float decayedDiagInfluence = (float)std::exp(-M_SQRT2 * decay);

    // Iterate over each tile on the map
    for (int row = 0; row < mapSize; row++) {
        for (int col = 0; col < mapSize; col++) {
            if (terrain->is_wall(row, col)) {  // Walls do not propagate influence
                tempLayer[row][col] = 0.0f;
                continue;
            }

            float maxNeighborInfluence = 0.0f;

            for (auto [i, j] : neighbors) {
                int neighborRow = row + i;
                int neighborCol = col + j;
                if (terrain->is_valid_grid_position(neighborRow, neighborCol) &&
                    is_clear_path(row, col, neighborRow, neighborCol)) {
                    float decayedInfluence = layer.get_value(neighborRow, neighborCol) * std::exp(-decay);
                    maxNeighborInfluence = std::max(maxNeighborInfluence, decayedInfluence);
                }
            }

            for (auto [i, j] : diagonals) {
            	int neighborRow = row + i;
				int neighborCol = col + j;
				if (terrain->is_valid_grid_position(neighborRow, neighborCol) &&
					is_clear_path(row, col, neighborRow, neighborCol)) {
					float decayedInfluence = layer.get_value(neighborRow, neighborCol) * decayedDiagInfluence;
					maxNeighborInfluence = std::max(maxNeighborInfluence, decayedInfluence);
				}
            }

            // Store the result in the current temp layer
            tempLayer[row][col] = oneMinusGrowth * layer.get_value(row, col) + growth * maxNeighborInfluence;
        }
    }

    // Write results from the final temp layer to the original layer
    for (int row = 0; row < mapSize; row++) {
        for (int col = 0; col < mapSize; col++) {
            layer.set_value(row, col, tempLayer[row][col]); // Use the updated temp layer
        }
    }
}

void propagate_dual_occupancy(MapLayer<float> &layer, float decay, float growth)
{
    /*
        Similar to the solo version, but the values range from -1.0 to 1.0, instead of 0.0 to 1.0

        For every cell in the given layer:

        1) Get the value of each neighbor and apply decay factor
        2) Keep the highest ABSOLUTE value from step 1
        3) Linearly interpolate from the cell's current value to the value from step 2
           with the growing factor as a coefficient.  Make use of the lerp helper function.
        4) Store the value from step 3 in a temporary layer.
           A float[40][40] will suffice, no need to dynamically allocate or make a new MapLayer.

        After every cell has been processed into the temporary layer, write the temporary layer into
        the given layer;
    */

    // WRITE YOUR CODE HERE
    //float tempLayer[Terrain::maxMapHeight][Terrain::maxMapWidth];

    //// Iterate over each tile on the map
    //for (int row = 0; row < mapSize; row++) {
    //    for (int col = 0; col < mapSize; col++) {
    //        float highestAbsNeighborValue = 0.0f; // Store the highest absolute value

    //        // Get the influence value of each neighbor after decay
    //        for (int i = -1; i <= 1; i++) {
    //            for (int j = -1; j <= 1; j++) {
    //                int neighborRow = row + i;
    //                int neighborCol = col + j;
    //                if ((i == 0 && j == 0) || !terrain->is_valid_grid_position(neighborRow, neighborCol)) {
    //                    continue; // Skip current cell and invalid neighbors
    //                }

    //                // Calculate distance for decay factor
    //                float distance = (i == 0 || j == 0) ? 1.0f : (float)M_SQRT2;
    //                float neighborValue = layer.get_value(neighborRow, neighborCol) * std::exp(-distance * decay);

    //                // Update highestAbsNeighborValue if the absolute value is higher
    //                highestAbsNeighborValue = std::max(highestAbsNeighborValue, std::abs(neighborValue));
    //            }
    //        }

    //        // Determine the sign of the highest neighbor value to maintain it in interpolation
    //        float highestNeighborValue = (highestAbsNeighborValue == 0.0f) ?
    //            0.0f : highestAbsNeighborValue * (layer.get_value(row, col) >= 0.0f ? 1.0f : -1.0f);

    //        // Apply linear interpolation with growing factor as coefficient
    //        float newValue = lerp(layer.get_value(row, col), highestNeighborValue, growth);
    //        tempLayer[row][col] = newValue;
    //    }
    //}

    //// Copy values from tempLayer back to the original layer
    //for (int row = 0; row < terrain->get_map_height(); row++) {
    //    for (int col = 0; col < mapSize; col++) {
    //        layer.set_value(row, col, tempLayer[row][col]);
    //    }
    //}
}

void normalize_solo_occupancy(MapLayer<float> &layer)
{
    /*
        Determine the maximum value in the given layer, and then divide the value
        for every cell in the layer by that amount.  This will keep the values in the
        range of [0, 1].  Negative values should be left unmodified.
    */

    // WRITE YOUR CODE HERE
    float maxValue = 0.0f; 
    const float mapSize = (float)terrain->get_map_width();
    
    // Find the maximum value in the layer
    for (int row = 0; row < mapSize; ++row) {
        for (int col = 0; col < mapSize; ++col) {
            //Only consider positive values
            if (layer.get_value(row, col) > 0.0f) {
                maxValue = std::max(maxValue, layer.get_value(row, col)); 
            }
        }
    }

    // Normalize the positive values in the layer
    for (int row = 0; row < mapSize; ++row) {
        for (int col = 0; col < mapSize; ++col) {
            float cellValue = layer.get_value(row, col);
            if (cellValue > 0.0f) {
                layer.set_value(row, col, cellValue / maxValue); // Normalize
            }
        }
    }
}

void normalize_dual_occupancy(MapLayer<float> &layer)
{
    /*
        Similar to the solo version, but you need to track greatest positive value AND 
        the least (furthest from 0) negative value.

        For every cell in the given layer, if the value is currently positive divide it by the
        greatest positive value, or if the value is negative divide it by -1.0 * the least negative value
        (so that it remains a negative number).  This will keep the values in the range of [-1, 1].
    */

    // WRITE YOUR CODE HERE
    //float maxPositiveValue = 0.0f;
    //float minNegativeValue = 0.0f; // Will store the least negative value (closest to -1)
    //const float mapSize = (float)terrain->get_map_width();

    //// Find the maximum positive and minimum (most negative) values
    //for (int row = 0; row < terrain->get_map_height(); ++row) {
    //    for (int col = 0; col < mapSize; ++col) {
    //        float cellValue = layer.get_value(row, col);
    //        if (cellValue > 0.0f) {
    //            maxPositiveValue = std::max(maxPositiveValue, cellValue);
    //        }
    //        else if (cellValue < 0.0f) {
    //            minNegativeValue = std::min(minNegativeValue, cellValue); // Find the most negative value
    //        }
    //    }
    //}

    //// Normalize the values in the layer
    //for (int row = 0; row < terrain->get_map_height(); ++row) {
    //    for (int col = 0; col < mapSize; ++col) {
    //        float cellValue = layer.get_value(row, col);
    //        if (cellValue > 0.0f) {
    //            layer.set_value(row, col, cellValue / maxPositiveValue); // Normalize positive values
    //        }
    //        else if (cellValue < 0.0f) {
    //            layer.set_value(row, col, cellValue / -minNegativeValue); // Normalize negative values
    //        } // Values of 0.0 are left unchanged
    //    }
    //}
}

void enemy_field_of_view(MapLayer<float> &layer, float fovAngle, float closeDistance, float occupancyValue, AStarAgent *enemy)
{
    /*
        First, clear out the old values in the map layer by setting any negative value to 0.
        Then, for every cell in the layer that is within the field of view cone, from the
        enemy agent, mark it with the occupancy value.  Take the dot product between the view
        vector and the vector from the agent to the cell, both normalized, and compare the
        cosines directly instead of taking the arccosine to avoid introducing floating-point
        inaccuracy (larger cosine means smaller angle).

        If the tile is close enough to the enemy (less than closeDistance),
        you only check if it's visible to enemy.  Make use of the is_clear_path
        helper function.  Otherwise, you must consider the direction the enemy is facing too.
        This creates a radius around the enemy that the player can be detected within, as well
        as a fov cone.
    */

    // WRITE YOUR CODE HERE
    const float mapSize = (float)terrain->get_map_width();

    for (int row = 0; row < mapSize; row++) {
        for (int col = 0; col < mapSize; col++) {
            if (layer.get_value(row, col) < 0.0f) {
                layer.set_value(row, col, 0.0f); // Clear out old values
            }
        }
    }

    Vec3 enemyPos = enemy->get_position();
    GridPos enemyGridPos = terrain->get_grid_position(enemyPos);
    Vec3 enemyDir = enemy->get_forward_vector();
    enemyDir.Normalize();

    float cosFov = cosf(fovAngle/2 * (float)M_PI / 180.f); // Cos half the FOV angle
    float closedDistanceSquared = closeDistance * closeDistance;

    for (int row = 0; row < mapSize; row++) {
        for (int col = 0; col < mapSize; col++) {
            if (is_clear_path(enemyGridPos.row, enemyGridPos.col, row, col)) {
                GridPos cellPos = { enemyGridPos.row - row, enemyGridPos.col - col };
                float distance = (float)(cellPos.row * cellPos.row + cellPos.col * cellPos.col);

                Vec3 cPos = terrain->get_world_position(row, col);
                Vec3 toCell = cPos - enemyPos;

                if (distance < closedDistanceSquared) {
                    layer.set_value(row, col, occupancyValue);
                }
                else {
                    toCell.Normalize();
                    float dotProduct = enemyDir.Dot(toCell);

                    if (dotProduct > cosFov) {
                        layer.set_value(row, col, occupancyValue);
                    }
                }
            }
        }
    }
}

bool enemy_find_player(MapLayer<float> &layer, AStarAgent *enemy, Agent *player)
{
    /*
        Check if the player's current tile has a negative value, ie in the fov cone
        or within a detection radius.
    */

    const auto &playerWorldPos = player->get_position();

    const auto playerGridPos = terrain->get_grid_position(playerWorldPos);

    // verify a valid position was returned
    if (terrain->is_valid_grid_position(playerGridPos) == true)
    {
        if (layer.get_value(playerGridPos) < 0.0f)
        {
            return true;
        }
    }

    // player isn't in the detection radius or fov cone, OR somehow off the map
    return false;
}

bool enemy_seek_player(MapLayer<float> &layer, AStarAgent *enemy)
{
    /*
        Attempt to find a cell with the highest nonzero value (normalization may
        not produce exactly 1.0 due to floating point error), and then set it as
        the new target, using enemy->path_to.

        If there are multiple cells with the same highest value, then pick the
        cell closest to the enemy.

        Return whether a target cell was found.
    */

    // WRITE YOUR CODE HERE
    const float mapSize = (float)terrain->get_map_width();
    float maxInfluence = 0.0f;
    GridPos targetPos = { -1, -1 };
    Vec3 enemyPos = enemy->get_position();

    for (int row = 0; row < mapSize; row++) {
        for (int col = 0; col < mapSize; col++) {
            float cellInfluence = layer.get_value(row, col);
            if (cellInfluence > maxInfluence) {
				maxInfluence = cellInfluence;
				targetPos = { row, col };
			}

            if (cellInfluence == maxInfluence && targetPos != GridPos{-1, -1}) {
				Vec3 targetWorldPos = terrain->get_world_position(row, col);
				Vec3 enemyToTarget = targetWorldPos - enemyPos;
				Vec3 currentTarget = terrain->get_world_position(targetPos.row, targetPos.col);
				Vec3 enemyToCurrent = currentTarget - enemyPos;

				if (enemyToTarget.Length() < enemyToCurrent.Length()) {
					targetPos = { row, col };
				}
			}
        }
    }

    if (targetPos.row != -1 && targetPos.col != -1) {
		enemy->path_to(terrain->get_world_position(targetPos.row, targetPos.col));
		return true;
	}

    return false; // REPLACE THIS
}
