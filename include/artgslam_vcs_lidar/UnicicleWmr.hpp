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
#include <cmath>
#include <tuple>

/**
 * @brief Simulates a unicycle model robot with position, velocity, and rendering support.
 */
class UnicicleWmr {
public:
    /**
     * @brief Constructor.
     * @param width_m Width of the robot in meters.
     * @param height_m Height of the robot in meters.
     * @param pixelsPerMeter Conversion factor to render robot dimensions in pixels.
     */
    UnicicleWmr(float width_m = 0.33f, float height_m = 0.28f, float pixelsPerMeter = 50.0f,int gridSize =500,float gridResolution = 0.1f);

    /**
     * @brief Updates the robot's position and orientation using its velocity.
     * @param dt Time step in seconds.
     */
    void update(float dt);

    /**
     * @brief Draws the robot on the screen.
     * @param window SFML render window.
     */
    void draw(sf::RenderWindow& window);

    /**
     * @brief Sets the linear and angular velocity of the robot.
     * @param line_v Linear velocity in m/s.
     * @param angular_omega Angular velocity in rad/s.
     */
    void setVelocity(float line_v, float angular_omega);

    /**
     * @brief Resets the robot pose and velocities.
     * @param x X position in meters.
     * @param y Y position in meters.
     * @param theta Orientation in radians.
     */
    void reset(float x = 0.0f, float y = 0.0f, float theta = 0.0f);

    // --- Getters for position and orientation ---
    float getX() const { return x; }
    float getY() const { return y; }
    float getTheta() const { return theta; }
    std::tuple<float, float, float> getPose() const {
        return std::make_tuple(x, y, theta);
    }
    // --- Getters for physical dimensions ---
    float getWidth() const { return width; }
    float getheight() const { return height; }

    // --- Activation state ---
    void setRobotActive(bool active) { isRobotActive = active; }
    bool getRobotActive() const { return isRobotActive; }

    // --- Setters for pose, color, and size ---
    void setPose(float x, float y, float theta);
    void setColor(sf::Color color);
    void setDimensions(float width, float height);
    sf::Vector2i wmrPos2Index(double x, double y);

private:
    float pixelsPerMeter; ///< Conversion factor from meters to pixels

    // Physical dimensions
    float width;  ///< Robot width in meters
    float height; ///< Robot height in meters

    // Pose
    float x;     ///< X position in meters
    float y;     ///< Y position in meters
    float theta; ///< Orientation in radians
    

    // Velocities
    float linear_v;      ///< Linear velocity (m/s)
    float angular_omega; ///< Angular velocity (rad/s)

    // SFML shape representing the robot
    sf::RectangleShape robotShape;
    sf::Color robotcolor; ///< Color of the robot shape

    // Activation state flag
    bool isRobotActive = false;

    int gridSize ;
    float gridResolution;
};
