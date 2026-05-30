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

#include <TGUI/Widgets/MenuBar.hpp>
#include <TGUI/Widgets/Label.hpp>
#include <TGUI/Widgets/ToggleButton.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>
#include <TGUI/TGUI.hpp>
#include <functional>

/**
 * @class MenuBar
 * @brief Manages the menu bar located at the bottom of the screen.
 * Provides options for file handling, view controls, robot creation, and live mode toggle.
 */
class MenuBar {
public:
    /**
     * @brief Constructor: initializes and places the menu elements within the given GUI.
     * @param gui Reference to the TGUI GUI object where menu elements will be placed.
     */
    MenuBar(tgui::Gui& gui);

    /**
     * @brief Sets the callback functions for each menu option:
     * - onOpen: Open a map file
     * - onSave: Save current map state
     * - onSaveImage: Export current map as an image
     * - onClose: Exit or close the application
     * - onResetView: Reset camera view
     * - onClearView: Clear the current grid/map
     * - onCreateRobot: Trigger robot creation interface
     * - onSimulation: Start or stop the simulation
     * 
     * @param onOpen Callback for "Open" menu option
     * @param onSave Callback for "Save" menu option
     * @param onSaveModMap Callback for "Save Image" menu option
     * @param onClose Callback for "Close" menu option
     * @param onResetView Callback for "Reset View" menu option
     * @param onClearView Callback for "Clear View" menu option
     * @param onCreateRobot Callback for "Create Robot" menu option
     * @param onSimulation Callback for "Simulation" menu option
     * @param onClearPath Callback for "Clear Path" menu option
     * @param onExecutePath Callback for "Execute Path" menu option
     * @param onPositionWindow
     * @param onResetLiveMode
     */
    void setCallbacks(
        std::function<void()> onOpen,
        std::function<void()> onSave,
        std::function<void()> onSaveModMap,
        std::function<void()> onClose,
        std::function<void()> onResetView,
        std::function<void()> onClearView,
        std::function<void()> onCreateRobot,
        std::function<void()> onSimulation,
        std::function<void()> onClearPath,
        std::function<void()> onActivateWmr,
        std::function<void()> onExecutePath,
        std::function<void()> onPositionWindow,
        std::function<void()>onRosNodeCtrl,
        std::function<void()>onMapCHessFill,
        std::function<void()>onMapFusion,
        std::function<void()>onLoggFusion
    );

    /**
     * @brief Updates the label showing the current grid coordinates of the mouse.
     * @param text The text to display (usually formatted coordinates)
     */
    void updateCoordinates(const std::string& text);

    /**
     * @brief Returns whether live mode is currently enabled.
     * @return true if live mode is active, false otherwise.
     */
    bool getLiveMode() const { return liveMode; };

private:
    /**
     * @brief Helper to set up the main menu bar structure and entries.
     * @param gui Reference to the TGUI GUI object.
     */
    void setupMenu(tgui::Gui& gui);

    /**
     * @brief Helper to set up the label that shows mouse grid coordinates.
     * @param gui Reference to the TGUI GUI object.
     */
    void setupCordLable(tgui::Gui& gui);

    /**
     * @brief Helper to set up the toggle button for live mode (ROS streaming).
     * @param gui Reference to the TGUI GUI object.
     */
    void setupliveToggle(tgui::Gui& gui);

    tgui::MenuBar::Ptr menuBar;           ///< Pointer to the TGUI MenuBar widget
    tgui::Label::Ptr coordLabel;          ///< Label to show mouse position in grid coordinates
    tgui::ToggleButton::Ptr liveToggle;   ///< Toggle button to activate/deactivate live mode

    bool liveMode = false;                ///< Indicates whether live mode is active
};
