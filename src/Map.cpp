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

#include <artgmap_vcs/Map.hpp>
#include <iostream> // For debugging/logging
#include "artgmap_vcs/Constants.hpp"

using namespace Constants;


/**
 * @brief Constructs a Map object with specified grid size and resolution.
 * Initializes the occupancy grid and links to the ViewController.
 * @param size Number of grid cells per side (square grid).
 * @param resolution Cell size in meters.
 * @param controller Reference to ViewController for visualization parameters.
 */
Map::Map(int size, double resolution, ViewController& controller)
    : gridSize(size), gridResolution(resolution), controller(controller), originSet(false)
{
    /// Initialize grid to all free cells (0)
    grid.resize(gridSize, std::vector<int>(gridSize, 0));
    logOddsGrid.resize(gridSize, std::vector<float>(gridSize,L_prior));
    manualObsBuffer.resize(gridSize, std::vector<int>(gridSize, 0));
    /// Clear stored real-world points
    posX.clear();
    posY.clear();

    /// Initialize goal and start index (use in A*)
    startIndex = {-1, -1};
    goalIndex = {-1, -1};
}

/**
 * @brief Adds a new real-world point to the list.
 * The first point added is used to set the origin for coordinate conversion.
 * @param x X coordinate in meters.
 * @param y Y coordinate in meters.
 */
void Map::addPoint(double x, double y)
{
    if (!originSet) {
        originX = x;
        originY = y;
        originSet = true;
    }
    posX.push_back(x);
    posY.push_back(y);
}

/**
 * @brief Replaces stored points with a new set.
 * @param newX Vector of X coordinates.
 * @param newY Vector of Y coordinates.
 */
void Map::setPoints(const std::vector<double>& newX, const std::vector<double>& newY)
{
    posX = newX;
    posY = newY;
}

/**
 * @brief Clears all stored real-world points and resets the origin flag.
 */
void Map::clearPoints()
{
    posX.clear();
    posY.clear();
    originSet = false;
}

/**
 * @brief Resets the occupancy grid cells to free (0), preserving start/goal.
 */
void Map::clearGrid()
{
    for (int row = 0; row < gridSize; ++row) {
        for (int col = 0; col < gridSize; ++col) {
            if (grid[row][col] != 's' && grid[row][col] != 'g') {
                grid[row][col] = 0;
            }
        }
    }
}

/**
 * @brief Converts real-world coordinates to grid indices relative to the origin.
 * Points outside the grid are ignored.
 * @param x Vector of X coordinates.
 * @param y Vector of Y coordinates.
 * @param xGrid Output vector of grid column indices.
 * @param yGrid Output vector of grid row indices.
 */
void Map::xy2Grid(const std::vector<double>& x, const std::vector<double>& y,
                 std::vector<int>& xGrid, std::vector<int>& yGrid) 
{
    xGrid.clear();
    yGrid.clear();
    xGrid.reserve(x.size());
    yGrid.reserve(y.size());
    
    // CRITICAL FIX: Find the center of your grid map space.
    // Since the robot is at 450, the center index is 450.
    int centerCellX = gridSize / 2; 
    int centerCellY = gridSize / 2; 

    for (size_t i = 0; i < x.size(); ++i) {
        // 1. Shift the metric coordinates by adding the center offset 
        // before dividing by the grid resolution factor
        int cellX = centerCellX + static_cast<int>(std::round(x[i] / gridResolution));
        int cellY = centerCellY + static_cast<int>(std::round(y[i] / gridResolution));

        // 2. Strict boundary protection check before pushing indices to the raycaster
        if (cellX >= 0 && cellX < gridSize && cellY >= 0 && cellY < gridSize) {
            xGrid.push_back(cellX);
            yGrid.push_back(cellY);
        }
    }
}
/**
 * @brief Fills the grid with obstacles at specified grid indices.
 * Previous occupancy data is cleared, but start/goal are preserved.
 * @param xGrid Vector of X (col) grid indices.
 * @param yGrid Vector of Y (row) grid indices.
 */
void Map::fillGrid(const std::vector<int>& xGrid, const std::vector<int>& yGrid)
{
    if (xGrid.size() != yGrid.size()) return;

    for (size_t i = 0; i < xGrid.size(); ++i) {
        int col = xGrid[i];
        int row = yGrid[i];

        if (col >= 0 && col < gridSize && row >= 0 && row < gridSize) {
            // Sets up obstacle only if cell not goal or start
            if (grid[row][col] != 's' && grid[row][col] != 'g') {
                grid[row][col] = 1; 
            }
        }
    }
}


/**
 * @brief Updates the occupancy grid based on the stored real-world points.
 */
void Map::updateGridFromPoints()
{
    if (posX.empty() || posY.empty()) return;

    std::vector<int> xGrid, yGrid;
    xy2Grid(posX, posY, xGrid, yGrid);
    fillGrid(xGrid, yGrid);
}

/**
 * @brief Draws occupied cells of the live map on the given SFML render target.
 * The drawing is centered and scaled according to the controller parameters.
 * @param target SFML RenderTarget to draw on.
 * @param color obstacle color 
 */
void Map::drawLiveMap(sf::RenderTarget& target, float pixelsPerMeter, sf::Color color) const
{
    if (grid.empty() || grid[0].empty()) return;

    const int rows = static_cast<int>(grid.size());
    const int cols = static_cast<int>(grid[0].size());

    const float cellSize = gridResolution * pixelsPerMeter;

    // Center the grid rendering around origin (0,0)
    const float offsetX = -(cols / 2.f) * cellSize;
    const float offsetY = -(rows / 2.f) * cellSize;

    sf::RectangleShape cellShape({cellSize, cellSize});

    for (int row = 0; row < rows; ++row)
    {
        for (int col = 0; col < cols; ++col)
        {
            const int val = grid[row][col];

            /// Assign colors based on cell value
            if (val == 1)
                cellShape.setFillColor(color);             /// obstacle (color set)
            else if (val == 's')
                cellShape.setFillColor(sf::Color::Red);    /// start
            else if (val == 'g')
                cellShape.setFillColor(sf::Color::Green);  /// goal
            else
                continue; /// skip free cells

            float x = offsetX + col * cellSize;
            float y = offsetY + row * cellSize;

            cellShape.setPosition(x, y);
            target.draw(cellShape);
        }
    }
    drawManualObs(target, pixelsPerMeter);
}


/**
 * @brief Marks a cell as the goal in the grid using ASCII code 'g' (103).
 * Removes previus goal.
 * @param col Column index.
 * @param row Row index.
 */
void Map::setGoal(int col, int row)
{
    /// Remove previous goal
    if (goalIndex.x >= 0 && goalIndex.y >= 0)
        grid[goalIndex.y][goalIndex.x] = 0;

    if (col >= 0 && row >= 0 && col < gridSize && row < gridSize) {
        grid[row][col] = 'g';
        goalIndex = {col, row}; /// Store current goal
    }
}

/**
 * @brief Checks if a cell is occupied or special.
 */
int Map::isOccupied(int col, int row) const
{
    if (col < 0 || row < 0 || col >= gridSize || row >= gridSize)
        return 1; // fuera de rango se considera ocupado

    const int val = grid[row][col];
    switch(val) {
        case 0: return 0;   // free
        case 1: return 1;   // obstacle
        case 's': return 's'; // start
        case 'g': return 'g'; // goal
        default: return 1;  // unexpected = occupied
    }
}

/**
 * @brief Clears a specific cell (free) by grid index.
 */
void Map::clearSetPoints(sf::Vector2i cellIndex)
{
    int col = cellIndex.x;
    int row = cellIndex.y;

    if (col >= 0 && row >= 0 && col < gridSize && row < gridSize)
        grid[row][col] = 0; 
}

/**
 * @brief Marks a cell as the start in the grid using ASCII code 's' (115).
 * Mueve automáticamente la celda previa si existe.
 * @param col Column index.
 * @param row Row index.
 */
void Map::setStart(int col, int row)
{
    std::cout << "[setStart] col=" << col << " row=" << row << '\n';
    if (grid[row][col] == 0) {
        grid[row][col] = 's'; // Mark cell as start
        startIndex = {col, row};
        std::cout << "Start set: " << grid[row][col] << std::endl;
    } else {
        std::cout << "[setStart] Cell is occupied." << std::endl;
    }
}

void Map::setStartOnfly(sf::Vector2i wmrIndexPos) 
{
    int col = wmrIndexPos.x;
    int row = wmrIndexPos.y;

    // Verificar índices válidos
    if (col < 0 || row < 0 || col >= gridSize || row >= gridSize) {
        return; // fuera de grid, no hacer nada
    }

    if (wmrIndexPos == startIndex) {
        return; // no cambio
    }

    // Si el nuevo start está en el goal, resetear goalIndex
    if (grid[row][col] == 'g') {
        goalIndex = {-1, -1};
    }

    // Liberar la celda anterior del start
    if (startIndex.x >= 0 && startIndex.y >= 0 &&
        startIndex.x < gridSize && startIndex.y < gridSize &&
        grid[startIndex.y][startIndex.x] == 's') {
        grid[startIndex.y][startIndex.x] = 0;
    }

    startIndex = wmrIndexPos;
    grid[row][col] = 's';
}

/**
 * @brief Toggles a single obstacle in the grid.
 */
void Map::setSingleObs(int col, int row)
{
    if (row < 0 || row >= gridSize || col < 0 || col >= gridSize)
        return;

    if (grid[row][col] == 's' || grid[row][col] == 'g')
        return; // no pisar start/goal

    if (manualObsBuffer[row][col] == 0) {
        grid[row][col] = 1;
        manualObsBuffer[row][col] = 1;
        std::cout << "Manual obstacle set" << std::endl;
    } else {
        grid[row][col] = 0;
        manualObsBuffer[row][col] = 0;
        std::cout << "Manual obstacle removed" << std::endl;
    }
}

void Map::drawManualObs(sf::RenderTarget& target, float pixelsPerMeter) const
{
    if (manualObsBuffer.empty()) return;

    const int rows = manualObsBuffer.size();
    const int cols = manualObsBuffer[0].size();
    const float cellSize = gridResolution * pixelsPerMeter;

    const float offsetX = -(cols / 2.f) * cellSize;
    const float offsetY = -(rows / 2.f) * cellSize;

    sf::RectangleShape cellShape({cellSize, cellSize});
    cellShape.setFillColor(sf::Color::Cyan); // obstáculo manual

    for (int row = 0; row < rows; ++row) {
        for (int col = 0; col < cols; ++col) {
            if (manualObsBuffer[row][col] == 1) {
                float x = offsetX + col * cellSize;
                float y = offsetY + row * cellSize;
                cellShape.setPosition(x, y);
                target.draw(cellShape);
            }
        }
    }
}

void Map::setObsGrid(int col,int row,int value){
    grid[row][col]=value;
}
std::vector<std::vector<int>> Map::getGridForAStar() const
{
    std::vector<std::vector<int>> combined = grid; // copia del grid principal

    for (int row = 0; row < gridSize; ++row) {
        for (int col = 0; col < gridSize; ++col) {
            // Añadir los obstáculos manuales
            if (manualObsBuffer[row][col] == 1) {
                combined[row][col] = 1;
            }
        }
    }

    return combined;
}

std::vector<std::pair<double,double>> Map::getPoints() const {
    std::vector<std::pair<double,double>> points;
    const auto& xs = getRealX();
    const auto& ys = getRealY();

    if(xs.size() != ys.size()) {
        std::cerr << "Warning: getRealX() y getRealY() tienen distinto tamaño\n";
        return points;
    }

    points.reserve(xs.size());
    for(size_t i = 0; i < xs.size(); ++i){
        points.emplace_back(xs[i], ys[i]);
    }
    return points;
}

void Map::possProcessMap(){

    for(int i=0; i<gridSize-1;i++){

        for(int j=0;j<gridSize-1;j++){

            if(grid[i][j] &&grid[i+1][j+1]&& !grid[i+1][j]&& !grid[i][j+1]){
                grid[i][j+1]=1;
                grid[i+1][j]=1;
            }
            if(!grid[i][j] &&!grid[i+1][j+1]&& grid[i+1][j]&& grid[i][j+1]){
                grid[i][j]=1;
                grid[i+1][j+1]=1;
            }
        }

    }
}

void Map::setGrid(const std::vector<std::vector<int>>& newGrid) {
    if (newGrid.empty()) {
        std::cerr << "[Map] Error: intento de asignar una grilla vacía.\n";
        return;
    }

    // Validar si el tamaño coincide con el mapa actual
    if (static_cast<int>(newGrid.size()) != gridSize ||
        static_cast<int>(newGrid[0].size()) != gridSize)
    {
        std::cerr << "[Map] Advertencia: el tamaño del nuevo grid no coincide con el actual. Redimensionando...\n";
        gridSize = static_cast<int>(newGrid.size());
        grid = newGrid; // se reasigna
    }
    else {
        grid = newGrid; // asignación directa
    }
}

// @brief Bresenham's algorithm (raycasting) it checks identifies every single discrete grid coordinate that the laser beam cleanly sliced through.
void Map::bresenham(sf::Vector2i robotCell, sf::Vector2i laserCell)
{
    // Safety check: Ensure the starting point is valid
    if (robotCell.x < 0 || robotCell.x >= gridSize || robotCell.y < 0 || robotCell.y >= gridSize) {
        std::cout << "[Bresenham Debug] ERROR: Robot start position (" 
                  << robotCell.x << ", " << robotCell.y << ") is OUT OF BOUNDS!" << std::endl;
        return;
    }

    int x = robotCell.x;
    int y = robotCell.y;

    int dx = std::abs(laserCell.x - robotCell.x);
    int dy = std::abs(laserCell.y - robotCell.y);

    int sx = (robotCell.x < laserCell.x) ? 1 : -1;
    int sy = (robotCell.y < laserCell.y) ? 1 : -1;

    int err = dx - dy;

    // Track statistics for this specific ray trace
    int freeCellsUpdated = 0;
    float initialStartLogOdds = logOddsGrid[y][x];

    // -------------------------------------------------------------------------
    // 1. Trace the ray path to apply FREE SPACE log-odds
    // -------------------------------------------------------------------------
    while (true) {
        // Stop right before we touch the actual endpoint obstacle cell
        if (x == laserCell.x && y == laserCell.y) {
            break;
        }

        // Apply Log-Odds Free Space update inside grid boundaries
        if (x >= 0 && x < gridSize && y >= 0 && y < gridSize) {
            logOddsGrid[y][x] += L_empty;
            if (logOddsGrid[y][x] < L_MIN) {
                logOddsGrid[y][x] = L_MIN;
            }
            freeCellsUpdated++;
        }

        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }

    // -------------------------------------------------------------------------
    // 2. Apply OCCUPIED log-odds to the final obstacle destination cell
    // -------------------------------------------------------------------------
    bool endpointUpdated = false;
    float finalObstacleLogOdds = 0.0f;

    if (laserCell.x >= 0 && laserCell.x < gridSize && laserCell.y >= 0 && laserCell.y < gridSize) {
        logOddsGrid[laserCell.y][laserCell.x] += L_occupie;
        if (logOddsGrid[laserCell.y][laserCell.x] > L_MAX) {
            logOddsGrid[laserCell.y][laserCell.x] = L_MAX;
        }
        endpointUpdated = true;
        finalObstacleLogOdds = logOddsGrid[laserCell.y][laserCell.x];
    }

    // -------------------------------------------------------------------------
    // 3. High-Level Summary Print (One line per ray)
    // -------------------------------------------------------------------------
    std::cout << "[Bresenham Debug] Ray: Robot(" << robotCell.x << "," << robotCell.y << ") -> "
              << "Laser(" << laserCell.x << "," << laserCell.y << ") | "
              << "Free updated: " << freeCellsUpdated << " cells | "
              << "Endpoint hit: " << (endpointUpdated ? "YES" : "OUT_OF_BOUNDS") 
              << " (New LogOdds: " << finalObstacleLogOdds << ")" << std::endl;
}

void Map::updateMapFromTransformedScan(sf::Vector2i robotGridPose, const std::vector<geometry_msgs::msg::Point32>& transformedLidarPoints)
{
    
    if (transformedLidarPoints.empty()) {
        std::cout << "[Map Debug] WARNING: Received an empty laser scan message." << std::endl;
        return;
    }

    // 1. Prepare batch vectors ONLY for the lidar endpoints
    static std::vector<double> rawX;
    static std::vector<double> rawY;
    
    rawX.clear();
    rawY.clear();
    rawX.reserve(transformedLidarPoints.size());
    rawY.reserve(transformedLidarPoints.size());

    for (const auto& point : transformedLidarPoints) {
        rawX.push_back(point.x);
        rawY.push_back(point.y);
    }

    // 2. Perform the coordinate transformation in ONE single batch operation
    std::vector<int> gridX;
    std::vector<int> gridY;
    xy2Grid(rawX, rawY, gridX, gridY);

    // --- DEBUG: Verify coordinate conversion output ---
    std::cout << "[Map Debug] After xy2Grid -> Converted Endpoints (X size: " 
              << gridX.size() << ", Y size: " << gridY.size() << ")" << std::endl;

    // Check for the Early Return Trap
    if (gridX.empty() || gridY.empty()) {
        std::cout << "[Map Debug] CRITICAL: Early exit triggered! Converted vectors are EMPTY. "
                  << "Check if your map size/resolution drops points outside its bounds." << std::endl;
        return;
    }

    // 3. Run Bresenham safely from index 0 up to the end of our valid converted points
    size_t validPointsCount = std::min(gridX.size(), gridY.size());
    size_t tracedRays = 0;

    // Print out a sample coordinate to make sure the transformation math makes sense
    if (validPointsCount > 0) {
        std::cout << "[Map Debug] Sample mapping trace: Real(" << rawX[0] << ", " << rawY[0] 
                  << ") -> Grid(" << gridX[0] << ", " << gridY[0] << ")" << std::endl;
    }

    for (size_t i = 0; i < validPointsCount; ++i) {
        sf::Vector2i laserCell(gridX[i], gridY[i]);

        // Boundary safety check for the raycaster
        if (laserCell.x >= 0 && laserCell.x < gridSize && laserCell.y >= 0 && laserCell.y < gridSize) {
            this->bresenham(robotGridPose, laserCell);
            tracedRays++;
        }
    }
    
    std::cout << "[Map Debug] Successfully traced " << tracedRays << " out of " << validPointsCount << " rays inside grid boundaries." << std::endl;

    // 4. Convert and log final grid state changes
    convertLogOddsToGrid();

    // Quick structural metric to see if cells are actually changing
    int occupiedCount = 0;
    for (int r = 0; r < gridSize; ++r) {
        for (int c = 0; c < gridSize; ++c) {
            if (grid[r][c] == 1) occupiedCount++;
        }
    }
    std::cout << "[Map Debug] Convert complete. Total active obstacles ('1') inside rendering grid: " << occupiedCount << std::endl;
}

sf::Vector2i Map::getCellIndices(double realX, double realY) const{

    double offset = (MAP_SIZE_CELLS * DEFAULT_METERS_PER_CELL) / 2.0; // Centered origin offset

    int col = static_cast<int>(std::floor((realX + offset) / gridResolution));
    int row = static_cast<int>(std::floor((realY + offset) / gridResolution));

    // Keep indices safely inside the memory grid boundaries
    col = std::clamp(col, 0, MAP_SIZE_CELLS - 1);
    row = std::clamp(row, 0, MAP_SIZE_CELLS - 1);

    return sf::Vector2i(col, row);
}

void Map::convertLogOddsToGrid() {
    // Ensure the main grid matches the dimensions of the logOddsGrid
    if (grid.size() != logOddsGrid.size()) {
        grid.assign(gridSize, std::vector<int>(gridSize, 0));
    }

    // Store current start and goal positions
    sf::Vector2i savedStart = startIndex;
    sf::Vector2i savedGoal = goalIndex;
    
    for (int r = 0; r < gridSize; ++r) {
        for (int c = 0; c < gridSize; ++c) {
            // Check if this is a start or goal cell from the stored indices
            bool isStartCell = (savedStart.x == c && savedStart.y == r);
            bool isGoalCell = (savedGoal.x == c && savedGoal.y == r);
            
            if (isStartCell) {
                grid[r][c] = 's';
                continue;
            }
            
            if (isGoalCell) {
                grid[r][c] = 'g';
                continue;
            }
            
            // Check if this is a manual obstacle (preserve it)
            if (r < manualObsBuffer.size() && c < manualObsBuffer[0].size() && 
                manualObsBuffer[r][c] == 1) {
                grid[r][c] = 1;
                continue;
            }
            
            // Convert log-odds to occupancy probability
            float lValue = logOddsGrid[r][c];
            float probability = 1.0f / (1.0f + std::exp(-lValue));
            
            // Apply threshold
            if (probability > 0.5f) {
                grid[r][c] = 1;  // Occupied
            } else {
                grid[r][c] = 0;  // Free
            }
        }
    }
    
    // Restore indices
    startIndex = savedStart;
    goalIndex = savedGoal;
    
    // CRITICAL: Sync the point storage with the updated grid
    syncPointsFromGrid();
}
/**
 * @brief Synchronizes the point storage (posX, posY) with the current grid.
 * Converts occupied grid cells back to world coordinates.
 */
void Map::syncPointsFromGrid() {
    posX.clear();
    posY.clear();
    
    // Reserve space for efficiency (max possible occupied cells)
    posX.reserve(gridSize * gridSize);
    posY.reserve(gridSize * gridSize);
    
    for (int row = 0; row < gridSize; ++row) {
        for (int col = 0; col < gridSize; ++col) {
            // Only save occupied cells (1), not start/goal
            if (grid[row][col] == 1) {
                // Convert grid indices back to world coordinates
                // This is the inverse of your xy2Grid transformation
                double worldX = (col - gridSize/2) * gridResolution;
                double worldY = (row - gridSize/2) * gridResolution;
                posX.push_back(worldX);
                posY.push_back(worldY);
            }
        }
    }
}