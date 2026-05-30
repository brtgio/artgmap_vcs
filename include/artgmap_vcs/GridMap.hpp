#pragma once
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

#include <vector>
#include <cmath>
#include <SFML/Graphics.hpp>
#include "artgmap_vcs/ViewController.hpp"

/**
 * @class GridMap
 * @brief Handles the creation and management of map data.
 * It stores (x, y) coordinates from sonar readings, converts them
 * into grid cell indices, and fills a 2D occupancy grid.
 */
class GridMap
{
private:
    int gridSize;                         ///< Number of cells in the grid (map size)
    double gridResolution;               ///< Size of each cell in meters
    std::vector<double> posX, posY;      ///< Real-world coordinates from sonar data
    std::vector<std::vector<int>> grid;  ///< 2D occupancy grid
    ViewController& controller;          ///< Reference to view controller (for proper rendering)
    bool isactive;                       ///<Variable to indicate if using simulation grid
    sf::Vector2i startIndex{0, 0};        ///< Current start position in grid indices
    sf::Vector2i goalIndex{-1, -1};       ///< Current goal position in grid indices
public:
 
    GridMap(int size, double resolution, ViewController& controller);

    /**
     * @brief Getters for real-world x coordinates
     * @return Vector of x coordinates
     */
    const std::vector<double>& getRealX() const { return posX; }

    /**
     * @brief Getters for real-world y coordinates
     * @return Vector of y coordinates
     */
    const std::vector<double>& getRealY() const { return posY; }

    int getCellIndexX(double realX) const;

    int getCellIndexY(double realY) const;

    void addPoints(double x, double y);

    void setPoints(const std::vector<double>& newX, const std::vector<double>& newY);

    void clearPoints();

    void setStart(int i, int j);

    void setGoal(int i, int j);

    int isOccupied(int i, int j) const;

    void xy2Grid(const std::vector<double>& x, const std::vector<double>& y,
                 std::vector<int>& xGrid, std::vector<int>& yGrid);

 
    void fillGrid(const std::vector<int>& xGrid, const std::vector<int>& yGrid);


    const std::vector<std::vector<int>>& getGrid() const;

    void clearGrid();

    void clearSetPoints(sf::Vector2i cellIndex);

    void draw(sf::RenderTarget& target, float pixelsPerMeter) const;

    int getMapSize() const { return gridSize; }
    float getGridResolution() const {return gridResolution;};

    void setActive(bool setActive){ isactive= setActive;};

    bool getIsActive() const {return isactive;};

    void setSingleObs(int i,int j);

    void possProcessMap();
    void setStartOnflyFromReal(double realX, double realY);

    void grid2XY();

};
