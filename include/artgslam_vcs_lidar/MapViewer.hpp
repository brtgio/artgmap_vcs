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


#include "artgslam_vcs_lidar/MenuBar.hpp"
#include "artgslam_vcs_lidar/FileManager.hpp"
#include "artgslam_vcs_lidar/MapRosConection.hpp"
#include "artgslam_vcs_lidar/ViewController.hpp"
#include "artgslam_vcs_lidar/RobotCreator.hpp"
#include "artgslam_vcs_lidar/UnicicleWmr.hpp"
#include "artgslam_vcs_lidar/Map.hpp"
#include "artgslam_vcs_lidar/RightClickMapMenu.hpp"
#include "artgslam_vcs_lidar/AStar.hpp"
#include "artgslam_vcs_lidar/DragWindow.hpp"
#include "artgslam_vcs_lidar/SaveMenu.hpp"
#include "artgslam_vcs_lidar/GridMap.hpp"
#include "artgslam_vcs_lidar/MapFusion.hpp"
#include "artgslam_vcs_lidar/Pose.hpp"
#include <SFML/Graphics.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>

/**
 * @class MapViewer
 * @brief This is the main class. It manages mouse/keyboard events and GUI integration.
 * It also coordinates the rendering and simulation components of the application.
 */
class MapViewer {
private:
    sf::RenderWindow& window;        ///< Reference to the SFML window for rendering

    sf::View view;                   ///< SFML camera view
    tgui::Gui gui;                   ///< GUI system for handling menus and widgets
    
    MenuBar menu;                   ///< Instance of the custom menu bar
    FileManager manager;           ///< Manages file loading/saving
    ViewController controller;     ///< Manages zoom, panning, grid drawing, and coordinate conversions
    GridMap map;                   ///< Represents and stores the occupancy grid
    AStar aStarsim;               ///< A* algorithm instance for path planning and animation
    MapRosConection rosconnect;         ///< Handles communication with ROS (publishing/subscribing)
    UnicicleWmr wmr;               ///< Simulated unicycle WMR (Wheeled Mobile Robot)
    Map livemap;              ///< Handles live mapping mode based on ROS data
    Map kinnectmap;
    Map fusionmap;
    RightClickMapMenu r_menu;     ///< Context menu for selecting start and goal positions
    DragWindow dragwin;           ///< Shows wmr pose when live mode is active

    bool running = true;               ///< Indicates whether the main loop is running
    sf::Vector2f worldXY;              ///< World coordinates (floating point)
    sf::Vector2i gridIndex;            ///< Current hovered grid cell
    sf::Vector2i gridIndex2copy;       ///< Temporary copy of a selected grid index
    sf::Vector2i wmrGridPose;          ///< Stores robot position on grid
    bool astarAnimating = false;       ///< Indicates whether the A* animation is currently running

public:
    auto getRosNode() { return rosconnect.get_node_base_interface(); }
    /**
     * @brief Constructor
     * @param win Reference to the SFML render window
     */
    MapViewer(sf::RenderWindow& win);

    /**
     * @brief Manages logic updates and user interaction
     */
    void update();

    /**
     * @brief Handles input events (mouse and keyboard)
     */
    void processEvent();

    /**
     * @brief Renders all visual components to the screen
     */
    void render();

    /**
     * @brief Returns whether the viewer should keep running
     * @return True if running, false otherwise
     */
    [[nodiscard]] bool isRunning() const;
};
