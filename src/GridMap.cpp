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

#include "artgmap_vcs/GridMap.hpp"
#include "artgmap_vcs/AStar.hpp"
#include <iostream>

/**
 * @brief Constructs a GridMap with given size, resolution, and stores reference to the controller,use for loading maps.
 * @param size Number of cells per grid side (grid is square).
 * @param resolution Size of each cell in meters.
 * @param controller Reference to the ViewController managing visualization.
 */
GridMap::GridMap(int size, double resolution, ViewController& controller)
    : gridSize(size), gridResolution(resolution), controller(controller),isactive(true)
{
    // Initialize the grid with all cells free (0)
    grid.assign(gridSize, std::vector<int>(gridSize, 0));
}

/**
 * @brief Converts real-world X coordinate to grid column index.
 * @param realX X coordinate in meters.
 * @return Grid column index corresponding to realX.
 */
int GridMap::getCellIndexX(double realX) const
{
    double offset = (gridSize * gridResolution) / 2.0; // Centered origin offset
    int j = static_cast<int>(std::floor((realX + offset) / gridResolution));
    j = std::clamp(j, 0, gridSize - 1);
    return j;
}

/**
 * @brief Converts real-world Y coordinate to grid row index.
 * @param realY Y coordinate in meters.
 * @return Grid row index corresponding to realY.
 */
int GridMap::getCellIndexY(double realY) const
{
    double offset = (gridSize * gridResolution) / 2.0;

    double val = (realY + offset) / gridResolution;
    double floored = std::floor(val);
    int before = static_cast<int>(floored);

    /*std::cout << "[Y DEBUG] realY=" << realY
              << " offset=" << offset
              << " val=" << val
              << " floor=" << floored
              << " index_before=" << before
              << std::endl;*/

    int i = static_cast<int>(std::floor((realY + offset) / gridResolution));
    i = std::clamp(i, 0, gridSize - 1);

    //std::cout << "[Y DEBUG] index_after=" << i << std::endl;

    return i;
}

/**
 * @brief Adds a single real-world point to internal storage.
 * @param x X coordinate in meters.
 * @param y Y coordinate in meters.
 */
void GridMap::addPoints(double x, double y)
{
    posX.push_back(x);
    posY.push_back(y);
}

/**
 * @brief Replaces stored points with new vectors of real-world coordinates.
 * @param newX Vector of X coordinates.
 * @param newY Vector of Y coordinates.
 */
void GridMap::setPoints(const std::vector<double>& newX, const std::vector<double>& newY)
{
    posX = newX;
    posY = newY;
}

/**
 * @brief Clears all stored real-world points.
 */
void GridMap::clearPoints()
{
    posX.clear();
    posY.clear();
}

/**
 * @brief Sets the start point on the grid if the cell is free.
 * @param col Column index in grid.
 * @param row Row index in grid.
 */
void GridMap::setStart(int col, int row)
{
    std::cout << "[setStart] col=" << col << " row=" << row << '\n';
    if (grid[row][col] == 0) {
        grid[row][col] = 's'; // Mark cell as start using ASCII code
        startIndex = {col, row};
        std::cout << "Start set: " << grid[row][col] << std::endl;
    } else {
        std::cout << "[setStart] Cell is occupied." << std::endl;
    }
}

/**
 * @brief Sets the goal point on the grid if the cell is free.
 * @param col Column index in grid.
 * @param row Row index in grid.
 */
void GridMap::setGoal(int col, int row)
{
    std::cout << "[setGoal] col=" << col << " row=" << row << '\n';
    if (grid[row][col] == 0) {
        grid[row][col] = 'g'; // Mark cell as goal using ASCII code
        goalIndex = {col, row};
        std::cout << "Goal set: " << grid[row][col] << std::endl;
    } else {
        std::cout << "[setGoal] Cell is occupied." << std::endl;
    }
}

/**
 * @brief Checks if a grid cell is occupied or special.
 * @param i Column index in grid.
 * @param j Row index in grid.
 * @return 1 if obstacle, 0 if free, 's' for start, 'g' for goal, else 1 (occupied).
 */
int GridMap::isOccupied(int i, int j) const
{
    if (grid[j][i] == 1) {
        return 1; // Obstacle
    }
    if (grid[j][i] == 0) {
        return 0; // Free cell
    }
    if (grid[j][i] == 's') {
        return 's'; // Start
    }
    if (grid[j][i] == 'g') {
        return 'g'; // Goal
    }
    // Default to occupied for unexpected values
    return 1;
}

/**
 * @brief Converts vectors of real-world coordinates to grid indices.
 * @param x Vector of X coordinates.
 * @param y Vector of Y coordinates.
 * @param xGrid Output vector of grid column indices.
 * @param yGrid Output vector of grid row indices.
 */
void GridMap::xy2Grid(const std::vector<double>& x, const std::vector<double>& y,
                      std::vector<int>& xGrid, std::vector<int>& yGrid)
{
    const int halfGrid = gridSize / 2;
    size_t n = std::min(x.size(), y.size());

    xGrid.resize(n);
    yGrid.resize(n);

    for (size_t i = 0; i < n; ++i) {
        int gx = static_cast<int>(std::round(x[i] / gridResolution));
        int gy = -static_cast<int>(std::round(y[i] / gridResolution));

        // Translate to centered origin indices
        xGrid[i] = gx + halfGrid;
        yGrid[i] = gy + halfGrid;
    }
}

/**
 * @brief Fills grid cells with obstacles based on given grid indices.
 * @param xGrid Vector of grid column indices.
 * @param yGrid Vector of grid row indices.
 */
void GridMap::fillGrid(const std::vector<int>& xGrid, const std::vector<int>& yGrid)
{
    // Reset grid to free cells
    grid.assign(gridSize, std::vector<int>(gridSize, 0));

    for (size_t i = 0; i < xGrid.size(); ++i) {
        int x = xGrid[i];
        int y = yGrid[i];

        if (x >= 0 && x < gridSize && y >= 0 && y < gridSize) {
            grid[y][x] = 1; // Mark obstacle
        }
    }
}

/**
 * @brief Returns a const reference to the internal grid representation.
 * @return Reference to grid.
 */
const std::vector<std::vector<int>>& GridMap::getGrid() const
{
    return grid;
}

/**
 * @brief Clears all cells in the grid, setting them to free (0).
 */
void GridMap::clearGrid()
{
    grid.assign(gridSize, std::vector<int>(gridSize, 0));
}

/**
 * @brief Clears a specific grid cell by setting it free (0).
 * @param cellIndex Grid cell index to clear.
 */
void GridMap::clearSetPoints(sf::Vector2i cellIndex)
{
    if (cellIndex.x >= 0 && cellIndex.y >= 0 &&
        cellIndex.y < gridSize && cellIndex.x < gridSize) {
        grid[cellIndex.y][cellIndex.x] = 0;
    }
}

/**
 * @brief Draws the grid cells with obstacles, start and goal points on an SFML render target.
 * @param target SFML RenderTarget to draw on.
 * @param pixelsPerMeter Scaling factor for rendering.
 */
void GridMap::draw(sf::RenderTarget& target, float pixelsPerMeter) const
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

            // Assign colors based on cell value
            if (val == 1)
                cellShape.setFillColor(sf::Color::Yellow); // obstacle
            else if (val == 's')
                cellShape.setFillColor(sf::Color::Red);    // start
            else if (val == 'g')
                cellShape.setFillColor(sf::Color::Green);  // goal
            else
                continue; // skip free cells

            float x = offsetX + col * cellSize;
            float y = offsetY + row * cellSize;

            cellShape.setPosition(x, y);
            target.draw(cellShape);
        }
    }
}
void GridMap::setSingleObs(int i ,int j){
       
    if (grid[j][i] == 's' || grid[j][i] == 'g') {
        return;
    }

    // Toggle 0 <-> 1
    if (grid[j][i] == 0) {
        grid[j][i] = 1;
    } else if (grid[j][i] == 1) {
        grid[j][i] = 0;
    }
}
        

void GridMap::possProcessMap(){

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
void GridMap::setStartOnflyFromReal(double realX, double realY)
{
    int col = getCellIndexX(realX);
    int row = getCellIndexY(realY);

    // Verificar índices válidos
    if (col < 0 || row < 0 || col >= gridSize || row >= gridSize) {
        return; // fuera de grid, no hacer nada
    }

    if (sf::Vector2i(col, row) == startIndex) {
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

    startIndex = {col, row};
    grid[row][col] = 's';
}

void GridMap::grid2XY()
{
    posX.clear();
    posY.clear();

    const double offset = (gridSize * gridResolution) / 2.0;

    for (int row = 0; row < gridSize; ++row) {
        for (int col = 0; col < gridSize; ++col) {

            if (grid[row][col] == 1) {

                double x = (col + 0.5) * gridResolution - offset;
                double y = (row + 0.5) * gridResolution - offset;

                posX.push_back(x);
                posY.push_back(y);
            }
        }
    }
}