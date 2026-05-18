/* 
 * -----------------------------------------------------------------------------
 *  Copyright (c) 2025 Gilberto Ramos Valenzuela
 *
 *  This file is part of the Artgslam Visualizer project.
 *
 *  Licensed for personal, academic, and non-commercial use only.
 *  Commercial use of the complete application "Artgslam Visualizer" is prohibited
 *  without explicit permission from the copyright holder.
 *
 *  For full license details, see the LICENSE.txt file distributed with this software.
 * -----------------------------------------------------------------------------
 */

#include "artgslam_vcs_lidar/AStar.hpp"
#include <iostream>
#include <cmath>
#include <algorithm> // std::reverse
#include <limits>
#include <unistd.h>


/**
 * @brief Constructs an AStar pathfinding object.
 *
 * Initializes internal data structures such as state maps, cost matrices,
 * parent tracking, and flags required to run the A* algorithm. The grid map,
 * live map, and fused map references are stored for use during search.
 *
 * @param mapRef Reference to the main grid map used for navigation.
 * @param liveref Reference to the live (sensor-based) map.
 * @param fusemapRef Reference to the fused map combining live and static data.
 */
AStar::AStar(GridMap& mapRef, Map& liveref, Map& fusemapRef)
    : map(mapRef), livemap(liveref), fusemap(fusemapRef)
{
    width = map.getMapSize(); // Map width (assuming square map)
    height = width;

    // Initialize stateMap with all cells empty
    stateMap.resize(height, std::vector<State>(width, State::cEmpty));

    // Initialize closed list with false flags (unvisited)
    closedList.resize(height, std::vector<bool>(width, false));

    inOpenList.resize(height, std::vector<bool>(width, false));

    // Initialize parent map with invalid parent indices
    parentMap.resize(height, std::vector<sf::Vector2i>(width, {-1, -1}));

    // Initialize gScores to infinity (unexplored)
    gScore.resize(height, std::vector<float>(width, std::numeric_limits<float>::infinity()));

    // Initialize start and goal indices as invalid
    startIndex = {-1, -1};
    goalIndex = {-1, -1};

    pathFound = false; // No path found yet
    finished = false;  // Algorithm not finished yet
    finalPath.clear(); // Clear any previous path

    distanceMap.resize(height, std::vector<int>(width, 1000));
}

/**
 * @brief Returns valid neighbors around a node in 8 directions.
 * @param node Current node.
 * @return Vector of neighbor cell coordinates.
 */
std::vector<sf::Vector2i> AStar::getNeighbors(const Node& node) const {
    // Direcciones: N, NE, E, SE, S, SW, W, NW
    static const std::vector<sf::Vector2i> directions = {
        { 0, -1}, // North
        { 1, -1}, // North-East
        { 1,  0}, // East
        { 1,  1}, // South-East
        { 0,  1}, // South
        {-1,  1}, // South-West
        {-1,  0}, // West
        {-1, -1}  // North-West
    };
    
    std::vector<sf::Vector2i> neighbors;
    
for (const auto& dir : directions) {
    int nx = node.x + dir.x;
    int ny = node.y + dir.y;
    if (isValidCell(nx, ny) ) {
        neighbors.emplace_back(nx, ny);
    }
}
    
    return neighbors;
}


/**
 * @brief Initializes the pathfinding process.
 * Resets all data structures and prepares the start node.
 */
void AStar::start() {
    // First, update the map and rebuild the stateMap
    updatemap();

    // Check that start and goal are properly defined
    if (startIndex.x == -1 || goalIndex.x == -1) {
        std::cerr << "Start or Goal not set properly.\n";
        finished = true;
        return;
    }

    // Reset pathfinding structures
    closedList.assign(height, std::vector<bool>(width, false));
    inOpenList.assign(height, std::vector<bool>(width, false));  // ← ADD THIS
    parentMap.assign(height, std::vector<sf::Vector2i>(width, {-1, -1}));
    gScore.assign(height, std::vector<float>(width, std::numeric_limits<float>::infinity()));
    openList = {}; // Clear priority queue

    // Initialize start node
    Node startNode(startIndex.x, startIndex.y);
    inOpenList[startIndex.y][startIndex.x] = true;
    startNode.gCost = 0;
    startNode.hCost = octileHeuristic(startIndex.x, startIndex.y, goalIndex.x, goalIndex.y);

    openList.push(startNode);
    gScore[startIndex.y][startIndex.x] = 0.0f;

    pathFound = false;
    finished = false;
}



/**
 * @brief Performs one iteration (step) of the A* algorithm.
 * 
 * This function executes a single expansion step of the A* search:
 * - Extracts the best (lowest f-cost) node from the open list.
 * - Checks if it has been visited before.
 * - Marks the node as closed.
 * - Checks whether the goal has been reached.
 * - Expands valid neighbors, updates their costs, and inserts them into the open list.
 * 
 * @return true  If more steps are required to finish the search.
 * @return false If the algorithm has finished (successfully or unsuccessfully).
 */
bool AStar::step() {
    float delay = 0;

    // If the algorithm is already finished, no more work to do
    if (finished) {
        sleep(delay);
        return false;
    }

    // If the open list is empty, there is no valid path
    if (openList.empty()) {
        finished = true;
        pathFound = false;
        sleep(delay);
        return false;
    }

    // Retrieve the node with the lowest f-cost and remove it from the open list
    currentNode = openList.top();
    openList.pop();
    inOpenList[currentNode.y][currentNode.x] = false;
    sleep(delay);

    // If this node was already processed (closed), skip it
    if (closedList[currentNode.y][currentNode.x]) {
        sleep(delay);
        return true;
    }

    // Mark node as closed (visited)
    closedList[currentNode.y][currentNode.x] = true;
    stateMap[currentNode.y][currentNode.x] = State::cClose;
    sleep(delay);

    // Goal check: if current node matches the goal coordinates
    if (currentNode.x == goalIndex.x && currentNode.y == goalIndex.y) {
        finished = true;
        pathFound = true;
        goalNode = currentNode;
        sleep(delay);

        // Reconstruct final path and post-process it
        finalPath = reconstructPath(goalNode);

        // Post-processing: simplify and optimize the path
        pathInflex = getInflexionPoints(finalPath);
        path_casthing = rayCasting(pathInflex);

        // Mark the path on the internal state map (excluding start and goal)
        for (const auto& pos : finalPath) {
            if (pos != startIndex && pos != goalIndex) {
                stateMap[pos.y][pos.x] = State::cPath;
                sleep(delay);
            }
        }

        return false; // Algorithm completed
    }

    // Select the active grid (live map or static map)
    std::vector<std::vector<int>> currentGrid;
    if (livemap.getIsActive()) {
        currentGrid = livemap.getGridForAStar();  // Combined real-time grid
    } else {
        currentGrid = map.getGrid();              // Static grid
    }

    // Process all valid neighboring cells
    for (const auto& neighborPos : getNeighbors(currentNode)) {
        int nx = neighborPos.x;
        int ny = neighborPos.y;

        // Ensure the neighbor is within bounds
        if (ny < 0 || ny >= static_cast<int>(currentGrid.size()) ||
            nx < 0 || nx >= static_cast<int>(currentGrid[0].size()))
            continue;

        // --- Corner-cutting prevention for diagonal movement ---
        int dx = nx - currentNode.x;
        int dy = ny - currentNode.y;

        // If diagonal movement is attempted, verify surrounding orthogonal cells
        if (std::abs(dx) == 1 && std::abs(dy) == 1) {
            int ortho1_x = currentNode.x + dx; // Horizontal adjacent cell
            int ortho1_y = currentNode.y;
            int ortho2_x = currentNode.x;      // Vertical adjacent cell
            int ortho2_y = currentNode.y + dy;

            // Ensure orthogonal cells are in bounds
            if (ortho1_x < 0 || ortho1_x >= static_cast<int>(currentGrid[0].size()) ||
                ortho1_y < 0 || ortho1_y >= static_cast<int>(currentGrid.size()) ||
                ortho2_x < 0 || ortho2_x >= static_cast<int>(currentGrid[0].size()) ||
                ortho2_y < 0 || ortho2_y >= static_cast<int>(currentGrid.size())) {
                continue; // Block unsafe diagonal
            }

            // If either orthogonal cell is occupied, diagonal is not allowed
            if (currentGrid[ortho1_y][ortho1_x] != 0 ||
                currentGrid[ortho2_y][ortho2_x] != 0) {
                continue;
            }
        }

        // Read occupancy of the current neighbor cell
        int occupancy = currentGrid[ny][nx];
        sleep(delay);

        // Skip if the cell is blocked or already closed
        if (closedList[ny][nx] || occupancy == 1) {
            continue;
        }

        // Determine if movement is diagonal and compute the movement cost
        bool isDiagonal = (std::abs(neighborPos.x - currentNode.x) == 1 && 
                           std::abs(neighborPos.y - currentNode.y) == 1);

        float moveCost = isDiagonal ? std::sqrt(2.0f) : 1.0f;

        // Compute tentative g-cost for this neighbor
        float tentativeG = currentNode.gCost + moveCost;
        //std::cout << "  Tentative gCost: " << tentativeG << "\n";
        sleep(delay);

        // If a better path to this neighbor is found
        if (tentativeG < gScore[ny][nx]) {
            gScore[ny][nx] = tentativeG;
            parentMap[ny][nx] = {currentNode.x, currentNode.y};

            // Add the neighbor to the open list if not already present
            if (!inOpenList[ny][nx]) {
                Node neighbor(nx, ny);
                neighbor.gCost = tentativeG;
                neighbor.hCost = octileHeuristic(nx, ny, goalIndex.x, goalIndex.y);

                openList.push(neighbor);
                inOpenList[ny][nx] = true; // Mark as queued
                sleep(delay);
            }
        }
    }

    return true; // Continue algorithm
}


/**
 * @brief Finds the complete path from start to goal.
 * @return Vector of cell positions along the path.
 */
std::vector<sf::Vector2i> AStar::findPath() {
    start();
    while (!finished) {
        if (!step()) break;
    }
    return pathFound ? reconstructPath(goalNode) : std::vector<sf::Vector2i>{};
    
}

/**
 * @brief Updates the start and goal positions by scanning the currently active map.
 *
 * This function selects the active grid (live map, fused map, or static map),
 * copies it into an internal buffer, and then inspects every cell to:
 * - Identify obstacles
 * - Detect the start ('s') and goal ('g') markers
 * - Update the internal state map used by the A* algorithm
 * 
 * After scanning, `startIndex` and `goalIndex` will contain the coordinates
 * of the start and goal nodes, or (-1, -1) if not found.
 */
void AStar::updatemap() {
    // Reset start and goal indices before scanning
    startIndex = {-1, -1};
    goalIndex  = {-1, -1};

    // Select the active grid source:
    //  - Live map (real-time sensor grid)      → highest priority
    //  - Fused map (combined information)      → second priority
    //  - Static map (original grid)            → fallback
    if (livemap.getIsActive()) {
        gridBuffer = livemap.getGridForAStar();  // Copy grid contents
    } 
    else if (fusemap.getIsActive()) {
        gridBuffer = fusemap.getGridForAStar();  // Copy fused grid
    }
    else {
        gridBuffer = map.getGrid();              // Copy static map grid
    }

    // activeGrid now points to the internal buffer safely stored in this class
    activeGrid = &gridBuffer;

    // Scan the entire map to classify each cell and find start/goal markers
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {

            int cell = (*activeGrid)[y][x];  // Read cell value

            if (cell == 1) {
                // Occupied cell → obstacle
                stateMap[y][x] = State::cObstacle;
            } 
            else if (cell == 's') {
                // Found start marker
                stateMap[y][x] = State::cStart;
                startIndex = {x, y};
            } 
            else if (cell == 'g') {
                // Found goal marker
                stateMap[y][x] = State::cGoal;
                goalIndex = {x, y};
            } 
            else {
                // Any other value → empty cell
                stateMap[y][x] = State::cEmpty;
            }
        }
    }
}





/**
 * @brief Reconstructs the path from the goal node back to the start node.
 * @param endNode The goal node from which to trace back.
 * @return Vector of cell positions representing the path.
 */
std::vector<sf::Vector2i> AStar::reconstructPath(const Node& endNode) const {
    std::vector<sf::Vector2i> path;
    sf::Vector2i current = {endNode.x, endNode.y};

    while (current != sf::Vector2i(-1, -1)) {
        path.push_back(current);
        current = parentMap[current.y][current.x];
    }

    std::reverse(path.begin(), path.end());
    
    return path;
}

/**
   * @brief Draws the current state of the map (open, closed, path, etc.) on the render target.
   * @param target SFML render target to draw on.
   * @param pixelsPerMeter Scaling factor for rendering.
   * @param metersPerCell Size of each grid cell in meters.
   */
void AStar::draw(sf::RenderTarget& target, float pixelsPerMeter, float metersPerCell) const {
    float cellSize = metersPerCell * pixelsPerMeter;
    float offsetX = -(width / 2.f) * cellSize;
    float offsetY = -(height / 2.f) * cellSize;

    sf::RectangleShape cellShape(sf::Vector2f(cellSize, cellSize));
    cellShape.setOutlineThickness(0);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            switch (stateMap[y][x]) {
                case State::cStart:
                    cellShape.setFillColor(sf::Color::Red);
                    break;
                case State::cEmpty:
                    continue; // Skip empty cells
                case State::cObstacle:
                    cellShape.setFillColor(sf::Color::Black);
                    break;
                case State::cClose:
                    cellShape.setFillColor(sf::Color(100, 149, 237, 180)); // Light blue, semi-transparent
                    break;
                case State::cPath:
                    cellShape.setFillColor(sf::Color::Yellow);
                    break;
                case State::cGoal:
                    cellShape.setFillColor(sf::Color::Green);
                    break;
                default:
                    continue;
            }

            cellShape.setPosition(offsetX + x * cellSize, offsetY + y * cellSize);
            target.draw(cellShape);
        }
    }
}

/**
 * @brief Draws the final path found by the algorithm.
 * @param target SFML render target.
 * @param pixelsPerMeter Pixels per meter scale.
 * @param metersPerCell Size of grid cell in meters.
 */
void AStar::drawFoundPath(sf::RenderTarget &target, float pixelsPerMeter, float metersPerCell) const {
    if (finalPath.empty()) return; // No path to draw

    float cellSize = metersPerCell * pixelsPerMeter;
    float offsetX = -(width / 2.f) * cellSize;
    float offsetY = -(height / 2.f) * cellSize;

    sf::RectangleShape cellShape(sf::Vector2f(cellSize, cellSize));
    cellShape.setFillColor(sf::Color::Blue);

    for (const auto& pos : finalPath) {
        if (pos == startIndex || pos == goalIndex) continue;

        float x = offsetX + pos.x * cellSize;
        float y = offsetY + pos.y * cellSize;
        cellShape.setPosition(x, y);

        target.draw(cellShape);
    }
}

/**
 * @brief Converts the final path from grid coordinates to metric (meters) coordinates.
 *
 * Uses the map resolution to transform each path point from cell indices
 * into real-world metric positions, storing the result in `finalPathMeters`.
 */
std::vector<sf::Vector2f> AStar::path2meters(const std::vector<sf::Vector2i>& path)
{
    std::vector<sf::Vector2f> pathInMeters;

    if (path.empty()) {
        std::cout << "no hay camino" << std::endl;
        return pathInMeters;
    }

    const float res = map.getGridResolution();
    const float halfRes = res / 2.0f;
    const float center = map.getMapSize() / 2.0f;  // Centro del mapa en celdas

    pathInMeters.reserve(path.size());

    for (std::size_t i = 0; i < path.size(); ++i) {
        // CORRECCIÓN: Aplicar halfRes ANTES de restar el centro
        // O mejor: calcular centro de celda en coordenadas relativas al origen
        
        // Método 1: Calcular centro de celda en coordenadas del mapa
        float x_center_in_map = (path[i].x - center) * res;
        float y_center_in_map = (path[i].y - center) * res;
        
        // Luego sumar halfRes para centrar dentro de la celda
        float x_m = x_center_in_map + halfRes;
        float y_m = -(y_center_in_map + halfRes);  // Invertir Y para el robot

        pathInMeters.emplace_back(x_m, y_m);

        std::cout << "Point " << i
                  << " (celda " << path[i].x << "," << path[i].y << ")"
                  << " -> x: " << x_m << " m, y: " << y_m << " m"
                  << std::endl;
    }

    return pathInMeters;
}


/**
 * @brief Clears all stored path data, including the final path and derived paths.
 */
void AStar::clearPath() {
    finalPath.clear();
    path_casthing.clear();
    pathInflex.clear();
}



/**
 * @brief Returns a path point from finalPathMeters, clamping the index to a valid range.
 *
 * Retrieves a point from the computed path in meters. If the index is outside
 * the valid range, it is clamped to the nearest valid value. If the path is empty,
 * a warning is printed and the returned point will be (0,0).
 *
 * @param index Requested index in the path.
 * @return sf::Vector2f The path point at the clamped index.
 *
 * @warning If finalPathMeters is empty, the function prints a warning and returns (0,0).
 * @note The index is automatically clamped to [0, finalPathMeters.size() - 1].
 */
sf::Vector2f AStar::getPathPoint(std::size_t index) const {
    // Case: empty path -> return a safe default and warn.
    if (finalPathMeters.empty()) {
        std::cerr << "[AStar::getPathPoint] WARNING: finalPathMeters is empty. Returning (0,0).\n";
    }

    // Clamp index to valid range [0, finalPathMeters.size() - 1].
    if (index < 0) {
        index = 0;
    }

    const std::size_t sz = finalPathMeters.size();
    if (static_cast<std::size_t>(index) >= sz) {
        index = static_cast<int>(sz) - 1;
    }

    return finalPathMeters[static_cast<std::size_t>(index)];
}


/**
 * @brief Checks whether a map cell can be safely occupied.
 *
 * Evaluates if the target cell and its surrounding safety margin are free
 * and within map boundaries. Returns true only if all checked cells are empty.
 */
bool AStar::canOccupy(int row, int col) const {
    if (!activeGrid) return false;

    int margin = 1; 
    if (row - margin < 0 || row + margin >= height ||
        col - margin < 0 || col + margin >= width)
        return false;

    for (int r = row - margin; r <= row + margin; ++r) {
        for (int c = col - margin; c <= col + margin; ++c) {
            if ((*activeGrid)[r][c] != 0) {
                return false;
            }
        }
    }

    return true;
}

/**
 * @brief Extracts all inflexion points from a given path.
 *
 * Scans the path and identifies every point where the movement direction
 * changes relative to the previous segment. The first point is always included,
 * and direction changes are detected by comparing consecutive (dx, dy) steps.
 * The goal index is appended at the end if it is not already included.
 *
 * @param path Reference to the full path expressed in grid coordinates.
 * @return A vector containing only the inflexion points.
 */
std::vector<sf::Vector2i> AStar::getInflexionPoints(std::vector<sf::Vector2i>& path) 
{
    std::vector<sf::Vector2i> inflexPoints;
    if (path.empty()) return inflexPoints; // No path → return empty list

    // The first point is always an inflexion point (path start)
    inflexPoints.push_back(path[0]);

    // Store the initial direction between the first two points
    sf::Vector2i prevDist{0,0};
    if (path.size() >= 2) {
        prevDist.x = path[1].x - path[0].x;
        prevDist.y = path[1].y - path[0].y;
    }

    // Iterate over the path and detect direction changes
    for (size_t i = 1; i < path.size(); ++i) {
        int dx = path[i].x - path[i - 1].x;  // Current step in x
        int dy = path[i].y - path[i - 1].y;  // Current step in y

        // A direction change occurs if the movement vector differs
        if (prevDist.x != dx || prevDist.y != dy) {
            std::cout << "Direction Change : "
                      << "( " << dx << ", " << dy << " )" << std::endl;

            // Add the current point as an inflexion point
            inflexPoints.push_back(path[i]);

            // Update previous direction
            prevDist.x = dx;
            prevDist.y = dy;
        } else {
            // Same direction → not an inflexion point
        }
    }

    // Ensure the goal point is included if not already the last inflexion
    if (path.back() != goalIndex) {
        inflexPoints.push_back(goalIndex);
    }

    return inflexPoints; 
}

/***
 * @brief Implements Raycasting algorithm using the inflextion points of a path
 * 
 */

std::vector<sf::Vector2i> AStar::rayCasting(std::vector<sf::Vector2i>& points)
{
    /* Optimized path that will store the simplified set of points
       after applying raycasting (line-of-sight pruning). */
    std::vector<sf::Vector2i> optimizePath;

    /* Start from the last point of the original path (usually the goal). */
    int i = points.size()-1;
    optimizePath.push_back(points[i]);

    /* Iterate backwards through the path while trying to connect
       the current point with the farthest previous point that still
       has line of sight. */
    while (i>0)
    {
        int foundIndex =-1;

        /* Loop backward to find the closest previous node (j)
           that has a clear line of sight to the current node (i). */
        for(int j=0; j<i;j++){

            /* If a straight line exists from points[i] to points[j],
               store j and break — we choose the farthest j possible. */
            if(checkLineOfSight(points[i],points[j])){
                foundIndex = j;
                break;
            }

        }

         /* If a valid point with line of sight was found,
           jump directly to it (skip intermediate nodes). */
        if(foundIndex !=-1){
            optimizePath.push_back(points[foundIndex]);
            i=foundIndex;
        }
        else{
             /* If no line of sight is found, step back normally
               to the previous point in the original path. */
            optimizePath.push_back(points[i-1]);
            i = i - 1;
        }
    }
        /* Reverse the optimized path because it was built backwards. */
    std::reverse(optimizePath.begin(),optimizePath.end());
        /* Append the goalIndex (assumed to be a final coordinate). */
    optimizePath.push_back(goalIndex);
    /* ----------------------------------------
       Debug print: original and optimized paths
       ---------------------------------------- */
    std::cout << "\nOriginal path:\n";
    for (auto& p : points)
        std::cout << "(" << p.x << "," << p.y << ")\n";

    std::cout << "\nOptimized path:\n";
    for (auto& p : optimizePath)
        std::cout << "(" << p.x << "," << p.y << ")\n";

        /* ---------------------------------------- */

    return optimizePath;


}

/**
 * @brief Checks line of sight between two points.
 *
 * Uses the point–slope form of a line to iterate through the cells between
 * the two coordinates and verify that no obstacles are present along the path.
 *
 * @param A Starting point of the line segment.
 * @param B Ending point of the line segment.
 *
 * @return true if there is a clear line of sight between A and B; false otherwise.
 */

bool AStar::checkLineOfSight(sf::Vector2i& A, sf::Vector2i& B) {

    /* If A and B form a vertical line, simply iterate along the Y direction
       and check every grid cell between them for obstacles. */
    if (A.x == B.x)
    {
        int start = std::min(A.y, B.y);
        int end   = std::max(A.y, B.y);

        for (int y = start; y <= end; y++)
        {
            if ((*activeGrid)[y][A.x] != 0)
                return false;
        }
        return true;
    }

    /* If A and B are not vertical, compute the slope (m) and intercept (b)
       using the point–slope form: y = m*x + b.
       These values allow us to trace the line between A and B and determine
       which grid cells the line passes through. */
    float m = float(B.y - A.y) / float(B.x - A.x);
    float b = A.y - m * A.x;

    /* If the slope is too steep (|m| > 1), iterate over the Y values, since
       stepping by X would skip grid cells. For each Y, compute the
       corresponding X value on the line and check that grid cell. */
    if (std::fabs(m) > 1.0f)
    {
        int start = std::min(A.y, B.y);
        int end   = std::max(A.y, B.y);

        for (int y = start; y <= end; y++)
        {
            float xf = (y - b) / m;
            int x = int(std::round(xf));

            if ((*activeGrid)[y][x] != 0)
                return false;
        }

        return true;
    }
    else
    {
        /* Normal case (|m| <= 1): iterate over X values.
           For each X, compute the corresponding Y on the line and check
           that grid cell for obstacles. */
        int start = std::min(A.x, B.x);
        int end   = std::max(A.x, B.x);

        for (int x = start; x <= end; x++)
        {
            float yf = m * x + b;
            int y = int(std::round(yf));

            if ((*activeGrid)[y][x] != 0)
                return false;
        }

        return true;
    }
}

/**
 * @brief Function that draws the optimized path after Raycasting optimization.
 *
 * Draws a polyline connecting all nodes in the simplified path and places
 * a filled circle at each node to make the path visually clear. The path
 * coordinates are converted from grid space to pixel space using the map's
 * scaling parameters.
 *
 * @param target Render target where the path will be drawn (window or texture).
 * @param pixelsPerMeter Conversion factor from meters to pixels.
 * @param metersPerCell Physical size of each grid cell, used to compute
 *                      the final pixel position of each node.
 */
void AStar::drawSimplifiPath(sf::RenderTarget& target, float pixelsPerMeter, float metersPerCell)
{
    if (path_casthing.empty()) return;

    float cellSize = metersPerCell * pixelsPerMeter;
    float offsetX  = -(width  / 2.f) * cellSize;
    float offsetY  = -(height / 2.f) * cellSize;

    const float halfCell = cellSize / 2.f; // offset al centro de la celda

    sf::VertexArray strip(sf::LineStrip, path_casthing.size());

    float r = std::max(1.f, cellSize * 0.15f);
    sf::CircleShape c(r);
    c.setOrigin(r, r);
    c.setFillColor(sf::Color::Magenta);

    for (size_t i = 0; i < path_casthing.size(); ++i)
    {
        sf::Vector2i idx = path_casthing[i];

        // Añadimos halfCell para centrar en la celda
        sf::Vector2f pos(
            offsetX + idx.x * cellSize + halfCell,
            offsetY + idx.y * cellSize + halfCell
        );

        strip[i].position = pos;
        strip[i].color    = sf::Color::White;

        c.setPosition(pos);
        target.draw(c);
    }

    target.draw(strip);
}







