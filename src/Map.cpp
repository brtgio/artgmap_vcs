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

#include <artgslam_vcs_lidar/Map.hpp>
#include <iostream> // For debugging/logging

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
    if (x.size() != y.size() || !originSet) return;

    xGrid.clear();
    yGrid.clear();

    int halfGrid = gridSize / 2;

    for (size_t i = 0; i < x.size(); ++i) {
        double shiftedX = x[i];
        double shiftedY = y[i];

        int col = static_cast<int>(std::round(shiftedX / gridResolution)) + halfGrid;
        int row = -static_cast<int>(std::round(shiftedY / gridResolution)) + halfGrid;

        // Ignore points outside grid bounds
        if (col < 0 || col >= gridSize || row < 0 || row >= gridSize)
            continue;

        xGrid.push_back(col);
        yGrid.push_back(row);
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