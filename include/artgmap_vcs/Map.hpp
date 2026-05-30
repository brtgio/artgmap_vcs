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
#include <math.h>
#include <SFML/Graphics.hpp>
#include "artgmap_vcs/ViewController.hpp"
#include "geometry_msgs/msg/point32.hpp"


/**
 * @class Map
 * @brief This class manages live map creation in "live mode", 
 * by dynamically updating the map from incoming ROS data (e.g., from RosHandler).
 * It builds an occupancy grid in real time and draws it using a ViewController.
 */
class Map {
public:
    /**
     * @brief Constructor: initializes map size, resolution and controller reference
     * @param size Number of grid cells per side
     * @param resolution Size of each grid cell in meters
     * @param controller Reference to the view controller
     */
    Map(int size, double resolution, ViewController& controller);

    /**
     * @brief Adds a single point (in real-world coordinates) to the internal point buffer
     * @param x Real-world X coordinate
     * @param y Real-world Y coordinate
     */
    void addPoint(double x, double y);

    /**
     * @brief Replaces current point buffer with a new set of coordinates
     * @param newX New set of X coordinates
     * @param newY New set of Y coordinates
     */
    void setPoints(const std::vector<double>& newX, const std::vector<double>& newY);

    /**
     * @brief Clears all stored real-world points
     */
    void clearPoints();

    /**
     * @brief Clears the occupancy grid (sets all cells to 0)
     * Preserves the start and goal cells
     */
    void clearGrid();
    
    /**
     * @brief Converts real-world coordinates to grid indices
     * @param x Vector of real-world X coordinates
     * @param y Vector of real-world Y coordinates
     * @param xGrid Output vector for grid column indices
     * @param yGrid Output vector for grid row indices
     */
    void xy2Grid(const std::vector<double>& x, const std::vector<double>& y,
                 std::vector<int>& xGrid, std::vector<int>& yGrid);

    /**
     * @brief Fills the grid using the given grid indices, marking cells as occupied (1)
     * Previous occupancy data is cleared but start/goal preserved
     * @param xGrid Vector of column indices
     * @param yGrid Vector of row indices
     */
    void fillGrid(const std::vector<int>& xGrid, const std::vector<int>& yGrid);

    /**
     * @brief Returns a constant reference to the full occupancy grid
     * @return Constant reference to grid
     */
    const std::vector<std::vector<int>>& getGrid() const { return grid; };

    /**
     * @brief Updates the grid from the internal point buffer
     */
    void updateGridFromPoints();

    /**
     * @brief Draws the live grid using the ViewController
     * @param target SFML render target
     * @param pixelsPerMeter Conversion factor from meters to pixels
     */
    void drawLiveMap(sf::RenderTarget& target, float pixelsPerMeter,sf::Color color) const;

    /**
     * @brief Activates or deactivates live mode (when true, dynamic updates occur)
     * @param isActive True to activate, false to deactivate
     */
    void setActive(bool isActive) { isLivemodeActive = isActive; };

    /**
     * @brief Returns whether live mode is currently active
     * @return True if live mode is active
     */
    bool getIsActive() const { return isLivemodeActive; };

    int getMapSize() const { return gridSize; }

    float getGridResolution() const {return gridResolution;};

    /**
     * @brief Sets a special start cell in the grid (value = 's')
     * Moves previous start cell back to free if necessary
     * @param col Grid column index
     * @param row Grid row index
     */
    void setStart(int col, int row);

    /**
     * @brief Sets a special goal cell in the grid (value = 'g')
     * Moves previous goal cell back to free if necessary
     * @param col Grid column index
     * @param row Grid row index
     */
    void setGoal(int col, int row);

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
    /**
     * @brief Query cell status.
     * @param col Column index
     * @param row Row index
     * @return 1 = occupied, 0 = free, ASCII 's' = start, ASCII 'g' = goal
     */
    int isOccupied(int col, int row) const;

    /**
     * @brief Clears a specific grid cell by setting it free (0)
     * @param cellIndex SFML vector containing column (x) and row (y)
     */
    void clearSetPoints(sf::Vector2i cellIndex);

    /**
     * @brief Sets a start cell directly from an SFML index
     * @param wmrPosIndex SFML vector containing column (x) and row (y)
     */
    void setStartOnfly(sf::Vector2i wmrPosIndex);

    /**
     * @brief Marks a single cell as an obstacle
     * @param col Grid column index
     * @param row Grid row index
     */
    void setSingleObs(int col, int row);
    void setObsGrid(int col,int row, int value);
    std::vector<std::vector<int>> getGridForAStar() const;

    std::vector<std::pair<double,double>> getPoints() const;

    void possProcessMap();
    void setGrid(const std::vector<std::vector<int>>& newGrid) ;
    void updateMapFromTransformedScan(sf::Vector2i robotGridPose, const std::vector<geometry_msgs::msg::Point32>& transformedLidarPoints);
    void bresenham(sf::Vector2i robotCell, sf::Vector2i laserCell);

    sf::Vector2i getCellIndices(double realX, double realY) const;

    void convertLogOddsToGrid();
    void syncPointsFromGrid();

private:
    ViewController& controller;           ///< Reference to the view controller for rendering
    int gridSize;                         ///< Map size (number of cells per side)
    double gridResolution;                ///< Size of each cell in real-world units

    std::vector<double> posX, posY;       ///< Buffer of real-world x/y points
    std::vector<std::vector<int>> grid;   ///< 2D occupancy grid (0 = free, 1 = occupied, 's' = start, 'g' = goal)
    std::vector<std::vector<float>> logOddsGrid; ///< 2d occupancy grid (keeping track of log odds instead of occupi and empty)
    std::vector<std::vector<int>> manualObsBuffer;///< Buffer to keep track of manual obstacles
    
    const float P_occupie = 0.7;
    const float P_empty = 0.4;
    /// @brief log-odds values calulated for probability 
    const float L_prior = 0; //Initial log-odds value of a grid cell when grid empty
    const float L_occupie = static_cast<float>(std::log(P_occupie/(1-P_occupie)));
    const float L_empty = static_cast<float>(std::log(P_empty/(1-P_empty)));

    /// Clamping thresholds to prevent numerical saturation/overflow
    const float L_MIN   = -5.0f; //< Strongly free space
    const float L_MAX   =  5.0f; //< Strongly occupied obstacle
    bool originSet = false;               ///< Flag to know if the origin is initialized
    double originX = 0.0, originY = 0.0;  ///< Optional offset for positioning

    bool isLivemodeActive = false;        ///< Whether live updates are happening

    sf::Vector2i startIndex{0, 0};        ///< Current start position in grid indices
    sf::Vector2i goalIndex{-1, -1};       ///< Current goal position in grid indices
    void drawManualObs(sf::RenderTarget& target, float pixelsPerMeter)  const;              ///< Fuction to draw manual obstacles from buffer
};
