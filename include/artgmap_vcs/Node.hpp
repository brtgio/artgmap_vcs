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

#include <SFML/Graphics.hpp>

/**
 * @class Node
 * @brief Represents a node in a grid used for path planning algorithms (e.g., A*).
 * Each node corresponds to a cell and holds all relevant data for cost calculation and path reconstruction.
 */
class Node {
private:
    // No private members in this version

public:
    /**
     * @brief Default constructor: initializes all coordinates and costs to invalid/default values.
     */
    Node() : x(-1), y(-1), parentX(-1), parentY(-1), gCost(0), hCost(0) {
        parent = sf::Vector2i(-1, -1);
    }

    /**
     * @brief Constructor with node position.
     * @param x Grid x-coordinate of the node.
     * @param y Grid y-coordinate of the node.
     */
    Node(int x, int y)
        : x(x), y(y), parentX(-1), parentY(-1), gCost(0), hCost(0)
    {
        parent = sf::Vector2i(parentX, parentY);
    }

    int x, y;                          ///< Current node (cell) position in grid coordinates
    sf::Vector2i parent;              ///< Parent node position (used for path reconstruction)
    int parentX, parentY;             ///< Redundant storage of parent coordinates
    float gCost;                      ///< Cost from the start node to this node
    float hCost;                      ///< Heuristic cost estimate from this node to the goal

    /**
     * @brief Returns the total estimated cost (f-cost) of the node: f = g + h.
     * This value is used in most path planning algorithms (e.g., A*) to prioritize nodes.
     * @return Sum of gCost and hCost.
     */
    float fCost() const { return gCost + hCost; }

    /**
     * @brief Comparison operator for use with priority queues (min-heap).
     * Returns true if this node has a higher f-cost than another node.
     * Nodes with lower f-costs are given higher priority.
     * @param other The other node to compare with.
     * @return true if this node's fCost is greater than the other's.
     */
    bool operator>(const Node& other) const {
        return this->fCost() > other.fCost();  // Solo f-cost, sin tie-breaker
    }
};
