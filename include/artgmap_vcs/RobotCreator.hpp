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


#include <iostream>
#include <SFML/Graphics.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>
#include <TGUI/Widgets/EditBox.hpp>
#include <TGUI/Widgets/Button.hpp>
#include <regex>
#include <cctype>

#include "artgmap_vcs/UnicicleWmr.hpp"

/**
 * @class RobotCreator
 * @brief Manages a GUI window for creating and configuring a Unicycle Wheeled Mobile Robot (WMR).
 * Allows user input for robot parameters like width, height, and color, then applies these to the robot model.
 */
class RobotCreator
{
public:
    /**
     * @brief Explicit constructor that requires a reference to the UnicicleWmr instance to configure.
     * The explicit keyword prevents unintended implicit conversions.
     * @param wmrRef Reference to the wheeled mobile robot instance
     */
    explicit RobotCreator(UnicicleWmr& wmrRef);

    /// Deleted default constructor to avoid creating an instance without a robot reference
    RobotCreator() = delete;

    /// Deleted copy constructor to prevent copying
    RobotCreator(const RobotCreator&) = delete;

    /// Deleted copy assignment operator to prevent copying
    RobotCreator& operator=(const RobotCreator&) = delete;

    /// Main loop entry point to run the creation window
    void run();

    /// Updates internal state and GUI elements each frame
    void update();

    /// Handles user input events like keyboard and mouse
    void processEvents();

    /// Renders the GUI window and widgets
    void render();

    /// Initializes and sets up all GUI widgets (EditBoxes, Buttons)
    void setupWidgets();

    /// Connects callbacks for button presses and other GUI events
    void setupCallbacks();

    /// Returns true if the creation window is currently open and running
    bool isRunning() const;

private:
    sf::RenderWindow windowRobotCreator; ///< SFML window for robot creation GUI
    tgui::Gui gui;                        ///< TGUI GUI manager attached to the window
    UnicicleWmr& wmr;                    ///< Reference to the robot being configured

    /// GUI widgets for robot parameters input
    tgui::EditBox::Ptr widthBox;  ///< Input for robot width
    tgui::EditBox::Ptr heightBox; ///< Input for robot height
    tgui::EditBox::Ptr colorBox;  ///< Input for robot color (hexadecimal string)

    /// Buttons for user actions
    tgui::Button::Ptr resetButton;  ///< Resets all input fields to default
    tgui::Button::Ptr createButton; ///< Applies parameters and creates/configures the robot

    /**
     * @brief Helper function to check if a string represents a valid floating-point number.
     * @param str Input string
     * @return true if valid float, false otherwise
     */
    bool isValidFloat(const std::string& str);

    /**
     * @brief Helper function to validate if a string is a valid hexadecimal color code.
     * Supports formats like "#RRGGBB" or "RRGGBB".
     * @param str Input string
     * @return true if valid hex color, false otherwise
     */
    bool isValidHexColor(const std::string& str);

    /**
     * @brief Converts a hexadecimal color string to an SFML Color object.
     * Assumes valid input format.
     * @param hex Hexadecimal color string
     * @return Corresponding sf::Color
     */
    sf::Color hexToColor(const std::string& hex);
};
