#pragma once
#include <TGUI/Widgets/MenuBar.hpp>
#include <TGUI/Widgets/Label.hpp>
#include <TGUI/Widgets/ToggleButton.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>
#include <TGUI/TGUI.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <functional>
#include <iostream>
#include <fstream>
#include "artgmap_vcs/Map.hpp"
#include "artgmap_vcs/UnicicleWmr.hpp"
#include "artgmap_vcs/tinyfiledialogs.h"

class SaveMenu {
private:
    Map& liveMap;
    Map& kinectMap;
    Map& fusionMap;
    UnicicleWmr& wmr;
    sf::RenderWindow saveWindow;
    tgui::Gui gui;
    std::string saveDirectory;

    tgui::Button::Ptr saveMultipleButton;
    tgui::Button::Ptr savePButton;
    tgui::Button::Ptr saveSButton;
    tgui::Button::Ptr saveDCButton;
    tgui::Button::Ptr saveFButton;

    tgui::CheckBox::Ptr cbWMR;
    tgui::CheckBox::Ptr cbSonar;
    tgui::CheckBox::Ptr cbDC;
    tgui::CheckBox::Ptr cbFusion;

public:
    SaveMenu(Map& livemap, Map& kinnectmap, Map& fusionmap, UnicicleWmr& wmr);
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

    void setupCallbacks();

};
