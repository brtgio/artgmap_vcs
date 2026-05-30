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


#include <cmath>
#include <queue>
#include <SFML/Graphics.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include "artgmap_vcs/Map.hpp"
#include "artgmap_vcs/Node.hpp"
#include "artgmap_vcs/GridMap.hpp"

/// \class AStar
/// \brief This class handles the A* algorithm for path planning.
/// It receives start and goal coordinates from the GridMap, performs
/// a step-by-step search, and provides path reconstruction and
/// visualization utilities.
class AStar {
public:
    /// \enum State
    /// \brief Enum representing the visual state of each grid cell.
    enum class State {
        cStart,     ///< Start cell
        cEmpty,     ///< Unvisited cell
        cObstacle,  ///< Obstacle cell
        cClose,     ///< Closed/visited cell
        cPath,      ///< Final path cell
        cGoal       ///< Goal cell
    };

    // --- Constructor ---
    AStar(GridMap& mapRef, Map& liveRef, Map& fusemapRef);

    // --- Algorithm control functions ---
    void start();
    bool step();
    std::vector<sf::Vector2i> findPath();
    void updatemap();

    // --- Visualization functions ---
    void draw(sf::RenderTarget& target, float pixelsPerMeter, float metersPerCell) const;
    void drawFoundPath(sf::RenderTarget& target, float pixelsPerMeter, float metersPerCell) const;

    // Getter functions (estos sí deben tener breve)
    /// \brief Returns whether the algorithm has finished executing.
    bool isFinished() const { return finished; }

    /// \brief Returns whether a path was successfully found.
    bool isPathFound() const { return pathFound; }

    /// \brief Gets the currently processed node (useful for debugging).
    const Node& getCurrentNode() const { return currentNode; }

    /// \brief Returns a reference to the state map for visualization.
    const std::vector<std::vector<State>>& getStateMap() const { return stateMap; }

    sf::Vector2f getPathPoint(std::size_t index) const;

    /// \brief Returns the size of the final path.
    const int getFinalPathSize() const { return finalPath.size(); }

    /// \brief Returns the size of the casted (raycasted) path.
    const int getCastPathSize() const { return path_casthing.size(); }

    // Other utilities (documentadas en el .cpp)
    std::vector<sf::Vector2f> path2meters(const std::vector<sf::Vector2i>& path);
    void clearPath();
    bool checkLineOfSight(sf::Vector2i& A, sf::Vector2i& B);
    void drawSimplifiPath(sf::RenderTarget& target, float pixelsPerMeter, float metersPerCell);
    std::vector<sf::Vector2i> getInflexionPoints(std::vector<sf::Vector2i>& path);
    std::vector<sf::Vector2i> rayCasting(std::vector<sf::Vector2i>& points);

    //Gether to get resulting paths
    const std::vector<sf::Vector2i>& getRawPath() const{ return finalPath;};
    const std::vector<sf::Vector2i>& getOptimizePath() const{return path_casthing;};
private:
    GridMap& map;      ///< Occupancy grid map (reference)
    Map& livemap;      ///< Live occupancy map reference
    Map& fusemap;

    std::vector<std::vector<int>> gridBuffer;
    const std::vector<std::vector<int>>* activeGrid = nullptr;

    // --- Internal state variables ---
    float gridResolution = map.getGridResolution();
    int width = 0;                    ///< Grid width
    int height = 0;                   ///< Grid height
    bool pathFound = false;           ///< True when a valid path exists
    bool finished = false;            ///< True once A* has completed

    sf::Vector2i startIndex = {-1, -1}; ///< Grid start index
    sf::Vector2i goalIndex  = {-1, -1}; ///< Grid goal index

    Node currentNode;
    Node goalNode;

    // Navigation helpers
    bool changeDesirePose = false;
    int DesireIndex = 0;

    int robotSize = 3;

    // --- A* structures ---
    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> openList;
    std::vector<std::vector<bool>> closedList;
    std::vector<std::vector<bool>> inOpenList;
    std::vector<std::vector<float>> gScore;
    std::vector<std::vector<sf::Vector2i>> parentMap;
    std::vector<std::vector<State>> stateMap;

    std::vector<sf::Vector2i> finalPath;
    std::vector<sf::Vector2i> pathInflex;
    std::vector<sf::Vector2i> path_casthing;
    std::vector<sf::Vector2f> finalPathMeters;

    std::vector<std::vector<int>> distanceMap;

    // --- Heuristic ---
    /// \brief Octile distance heuristic (ideal for 8-connected grids).
    double octileHeuristic(int x1, int y1, int x2, int y2) {
        int dx = std::abs(x2 - x1);
        int dy = std::abs(y2 - y1);
        return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
    }

    /// \brief Returns true if coordinates are within grid bounds.
    bool isValidCell(int x, int y) const {
        return x >= 0 && x < width && y >= 0 && y < height;
    }

    std::vector<sf::Vector2i> getNeighbors(const Node& node) const;
    std::vector<sf::Vector2i> reconstructPath(const Node& endNode) const;
    bool canOccupy(int row, int col) const;
};
