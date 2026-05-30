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
#include <TGUI/Backend/SFML-Graphics.hpp>
#include <TGUI/AllWidgets.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <regex>
#include "artgmap_vcs/Map.hpp"
#include "artgmap_vcs/GridMap.hpp"
/**
 * @class RightClickMapMenu
 * @brief Manages a contextual right-click menu displayed on top of a map.
 * Allows users to set the start and goal positions or clear the grid through TGUI buttons.
 */
class RightClickMapMenu {
public:
    /**
     * @brief Constructor: initializes the menu with references to the TGUI GUI system and map instances.
     * @param guiRef TGUI GUI object (must persist).
     * @param mapRef Reference to the static grid map.
     * @param livemapRef Reference to the live map (dynamic overlays, e.g., visited paths).
     */
    RightClickMapMenu(tgui::Gui& guiRef, GridMap &mapRef, Map& livemapRef,Map& fusemapRef);

    void setupWidgets();
    void show(float x, float y, const sf::Vector2i& gridIndex);
    void hide();
    void setVisible(bool show);
    bool isVisible() const;
    void connectSignals();
    bool containsPoint(const sf::Vector2f& point) const;

    // Nuevo: métodos para consultar si start y goal están activos
    bool isSetSim() const { return sim.startActive && sim.goalActive; }
    bool isSetLive() const { return live.startActive && live.goalActive; }

    // Devuelve el estado activo según el mapa
    

private:
    tgui::Gui& gui;
    GridMap& gridmap;
    Map& livemap;
    Map& fusemap;

    struct StartGoalState {
        sf::Vector2i start{-1, -1};
        sf::Vector2i goal{-1, -1};
        bool startActive = false;
        bool goalActive = false;
    };
    StartGoalState& getActiveState();

    StartGoalState sim;   ///< Estado del mapa de simulación
    StartGoalState live;  ///< Estado del mapa en tiempo real

    tgui::Group::Ptr container;
    tgui::ChildWindow::Ptr panel;

    tgui::Button::Ptr start;
    tgui::Button::Ptr goal;
    tgui::Button::Ptr clear;
    tgui::Button::Ptr clearView;

    bool visible = false;

    sf::Vector2f worldXY_copy;
    sf::Vector2i gridIndex_copy;

    // Struct interno para almacenar start/goal y su estado
    
};
