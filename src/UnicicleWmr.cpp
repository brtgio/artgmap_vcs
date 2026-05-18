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

#include "artgslam_vcs_lidar/UnicicleWmr.hpp"

/**
 * @brief Constructor.
 * 
 * Initializes the wheeled mobile robot with dimensions in meters and pixels-per-meter scale.
 * Sets initial position (0,0), orientation 0 radians, zero velocities, and default blue color.
 * 
 * @param width_m Width of the robot in meters.
 * @param height_m Height of the robot in meters.
 * @param pixelsPerMeter Scale factor converting meters to pixels.
 */
UnicicleWmr::UnicicleWmr(float width_m, float height_m, float pixelsPerMeter,
                         int gridSize, float gridResolution)
    : width(width_m), height(height_m), pixelsPerMeter(pixelsPerMeter),
      x(0.f), y(0.f), theta(0.f),
      linear_v(0.f), angular_omega(0.f),
      robotcolor(sf::Color(139, 0, 0)),
      gridSize(gridSize), gridResolution(gridResolution)
{
    /** Setup the SFML rectangle shape representing the robot with correct size and origin at center */
    robotShape.setSize(sf::Vector2f(width * pixelsPerMeter, height * pixelsPerMeter));
    robotShape.setOrigin(robotShape.getSize() / 2.f);
    robotShape.setFillColor(robotcolor);


}


/**
 * @brief Updates the robot's pose based on current velocities and elapsed time.
 * 
 * Simple unicycle kinematics: update x, y, and theta accordingly.
 * 
 * @param dt Time interval in seconds since last update.
 */
void UnicicleWmr::update(float dt)
{
    x += linear_v * cos(theta) * dt;
    y += linear_v * sin(theta) * dt;
    theta += angular_omega * dt;
}

/**
 * @brief Draws the robot on the given SFML render window if active.
 * 
 * Positions and rotates the shape based on current pose.
 * 
 * @param window Reference to the SFML render window.
 */
void UnicicleWmr::draw(sf::RenderWindow& window)
{
if (isRobotActive) {
        
        robotShape.setPosition(x * pixelsPerMeter ,
                               -y * pixelsPerMeter );

        robotShape.setRotation(theta * 180.f / 3.14159265f);  // rad → deg
        robotShape.setFillColor(robotcolor);
        window.draw(robotShape);
    }
}

/**
 * @brief Sets the linear and angular velocities of the robot.
 * 
 * @param linear_v Linear velocity (forward) in meters per second.
 * @param angular_omega Angular velocity (rotational) in radians per second.
 */
void UnicicleWmr::setVelocity(float linear_v, float angular_omega)
{
    this->linear_v = linear_v;
    this->angular_omega = angular_omega;
}

/**
 * @brief Resets the robot's pose to the specified position and orientation and stops movement.
 * 
 * @param x New x-position in meters.
 * @param y New y-position in meters.
 * @param theta New orientation in radians.
 */
void UnicicleWmr::reset(float x, float y, float theta)
{
    setPose(x, y, theta);
    setVelocity(0.f, 0.f);
}

/**
 * @brief Sets the robot's pose.
 * 
 * Updates the position (x, y) and orientation (theta).
 * 
 * @param x X-position in meters.
 * @param y Y-position in meters.
 * @param theta Orientation in radians.
 */
void UnicicleWmr::setPose(float x, float y, float theta)
{
    this->x = x;
    this->y = y;
    this->theta = theta;

}

/**
 * @brief Sets the color used for drawing the robot.
 * 
 * @param color SFML color to use.
 */
void UnicicleWmr::setColor(sf::Color color)
{
    robotcolor = color;
}

/**
 * @brief Updates the robot's dimensions and resizes the SFML shape accordingly.
 * 
 * Also resets the origin to the center for correct rotation and positioning.
 * 
 * @param width New width in meters.
 * @param height New height in meters.
 */
void UnicicleWmr::setDimensions(float width, float height)
{
    this->width = width;
    this->height = height;
    robotShape.setSize(sf::Vector2f(width * pixelsPerMeter, height * pixelsPerMeter));
    robotShape.setOrigin(robotShape.getSize() / 2.f);
}

/**
 * @brief Converts robot world position (x, y) to grid cell indices.
 *
 * The function maps the robot's continuous world coordinates into
 * discrete grid indices based on the grid resolution and the grid size.
 * If the computed index falls outside grid bounds, (-1, -1) is returned.
 *
 * @param x World coordinate in meters along the X-axis.
 * @param y World coordinate in meters along the Y-axis.
 * @return sf::Vector2i Grid indices (xIdx, yIdx) or (-1, -1) if out of bounds.
 */
sf::Vector2i UnicicleWmr::wmrPos2Index(double x, double y)
{
    int halfGrid = gridSize / 2;

    int xIdx = static_cast<int>(std::round(x / gridResolution)) + halfGrid;
    int yIdx = -static_cast<int>(std::round(y / gridResolution)) + halfGrid;

    /// Check grid boundaries
    if (xIdx < 0 || xIdx >= gridSize || yIdx < 0 || yIdx >= gridSize)
    {
        return {-1, -1}; /// Point is outside the grid
    }

    return {xIdx, yIdx};
}
