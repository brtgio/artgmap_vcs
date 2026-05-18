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

#include "artgslam_vcs_lidar/RightClickMapMenu.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>

/**
 * @brief Constructs the RightClickMapMenu.
 * 
 * Initializes references to GUI, GridMap, and LiveMap, then sets up the menu widgets
 * and connects button signals.
 * 
 * @param guiRef Reference to the TGUI GUI instance.
 * @param mapRef Reference to the GridMap instance.
 * @param livemapRef Reference to the LiveMap instance.
 */
RightClickMapMenu::RightClickMapMenu(tgui::Gui& guiRef, GridMap& mapRef, Map& livemapRef,Map& fusemapRef)
    : gui(guiRef), gridmap(mapRef), livemap(livemapRef),fusemap(fusemapRef)
{
    std::cout << "📦 RightClickMapMenu: loading..." << std::endl;
    setupWidgets();    /**< Load and configure GUI widgets */
    connectSignals();  /**< Connect button event handlers */
}

/**
 * @brief Loads GUI widgets from external TGUI form and applies styles.
 * 
 * Retrieves the ROS package path to locate the GUI form file, loads widgets into
 * a container, then retrieves and styles the individual buttons. Hides the panel by default.
 */
void RightClickMapMenu::setupWidgets()
{
    std::string package_path = ament_index_cpp::get_package_share_directory("artgslam_vcs_lidar");
    std::string formPath = package_path + "/assets/forms/Right_Click_Menu.txt";

    try {
        container = tgui::Group::create();
        container->loadWidgetsFromFile(formPath);
        gui.add(container);

        panel = container->get<tgui::ChildWindow>("ChildWindow1");
        if (!panel) {
            std::cerr << "❌ 'ChildWindow1' widget not found in form." << std::endl;
            return;
        }

        auto layout = panel->get<tgui::VerticalLayout>("VerticalLayout1");
        if (!layout) {
            std::cerr << "❌ 'VerticalLayout1' widget not found in panel." << std::endl;
            return;
        }

        start     = layout->get<tgui::Button>("start");
        goal      = layout->get<tgui::Button>("goal");
        clear     = layout->get<tgui::Button>("Clear");
        clearView = layout->get<tgui::Button>("ClearView");

        if (!start || !goal || !clear || !clearView) {
            std::cerr << "⚠️ One or more buttons not found in layout." << std::endl;
        } else {
            auto applyHoverStyle = [](tgui::Button::Ptr btn) {
                auto renderer = btn->getRenderer();
                renderer->setBackgroundColor(sf::Color::Transparent);
                renderer->setBackgroundColorHover(sf::Color(0, 120, 215)); // Windows 10 blue
                renderer->setTextColor(sf::Color::Black);
                renderer->setTextColorHover(sf::Color::White);
                renderer->setBorderColor(sf::Color::Transparent);
                renderer->setBorderColorHover(sf::Color(0, 120, 215));
                renderer->setBorders({1, 1, 1, 1}); // 1-pixel border
            };

            applyHoverStyle(start);
            applyHoverStyle(goal);
            applyHoverStyle(clear);
            applyHoverStyle(clearView);
        }

        panel->setVisible(false); /**< Hide panel initially */

    } catch (const tgui::Exception& e) {
        std::cerr << "❌ Failed to load TGUI form: " << e.what() << std::endl;
    }
}

/**
 * @brief Shows the right-click menu at specified screen coordinates with a selected grid index.
 * 
 * @param x Screen x position.
 * @param y Screen y position.
 * @param gridIndex Grid cell index selected.
 */
void RightClickMapMenu::show(float x, float y, const sf::Vector2i& gridIndex)
{
    if (!panel) return;

    gridIndex_copy = gridIndex;
    panel->setPosition(x, y);
    panel->setVisible(true);
    visible = true;
}

/**
 * @brief Hides the right-click menu.
 */
void RightClickMapMenu::hide()
{
    if (panel) {
        panel->setVisible(false);
        visible = false;
    }
}

/**
 * @brief Explicitly sets the visibility of the menu.
 * 
 * @param show True to show the menu, false to hide.
 */
void RightClickMapMenu::setVisible(bool show)
{
    if (panel) {
        panel->setVisible(show);
        visible = show;
    }
}

/**
 * @brief Returns whether the menu is currently visible.
 * 
 * @return true if visible, false otherwise.
 */
bool RightClickMapMenu::isVisible() const
{
    return visible;
}

/**
 * @brief Determines if a given point lies inside the menu boundaries.
 * 
 * @param point Point in screen coordinates to test.
 * @return true if point is inside menu bounds, false otherwise.
 */
bool RightClickMapMenu::containsPoint(const sf::Vector2f& point) const
{
    if (!panel) return false;

    sf::Vector2f pos = panel->getAbsolutePosition();
    sf::Vector2f size = panel->getSize();
    sf::FloatRect bounds(pos.x, pos.y, size.x, size.y);

    return bounds.contains(point);
}

/**
 * @brief Connects button press signals to their respective logic functions.
 * 
 * Handles setting start/goal points, clearing points, and other button actions.
 */
RightClickMapMenu::StartGoalState& RightClickMapMenu::getActiveState()
{
    if (livemap.getIsActive())
        return live;

    if (gridmap.getIsActive())
        return sim;

    return sim;
}

/**
 * @brief Connects all UI button signals for the right-click map menu.
 *
 * Associates Start, Goal, Clear, and ClearView buttons with their respective
 * callback handlers. These handlers update the internal state (start/goal points),
 * interact with the active map (LiveMap, GridMap, or FuseMap), and manage
 * visibility of the context menu.
 *
 * - **Start:**  
 *   - On LiveMap: Start is fixed and cannot be moved.  
 *   - On GridMap: Sets the start point at the clicked grid index.  
 *
 * - **Goal:**  
 *   - Only allows setting the goal if it is not active.  
 *   - Ensures exactly one map is active before assigning the goal.  
 *
 * - **Clear:**  
 *   - Clears Start and Goal points from the active map.  
 *
 * - **ClearView:**  
 *   - Placeholder button, currently only logs an action.  
 */
void RightClickMapMenu::connectSignals()
{
if (start) {
    start->onPress([this]() {
        auto& state = getActiveState();

        if (livemap.getIsActive()) {
            std::cout << "📍 Live map active: Start is fixed and cannot be moved." << std::endl;
            // Start siempre está activo en LiveMap, no hacemos nada
            state.startActive = true; 
        } else if (!state.startActive && gridmap.getIsActive()) {
            gridmap.setStart(gridIndex_copy.x, gridIndex_copy.y);
            std::cout << "📍 Start set on GridMap." << std::endl;

            state.start = gridIndex_copy;
            state.startActive = true;
        } else {
            std::cout << "⚠️ Start is already active or no map to set Start." << std::endl;
        }

        hide();
    });
}


    if (goal) {
        goal->onPress([this]() {
            auto& state = getActiveState();

            if (!state.goalActive) {
                 int activeCount = (livemap.getIsActive() ? 1 : 0)
                            + (gridmap.getIsActive() ? 1 : 0)
                            + (fusemap.getIsActive() ? 1 : 0);

                if (activeCount == 1) {
                    if (livemap.getIsActive()) {
                        livemap.setGoal(gridIndex_copy.x, gridIndex_copy.y);
                        std::cout << "🎯 Goal set on LiveMap." << std::endl;
                } else if (gridmap.getIsActive()) {
                        gridmap.setGoal(gridIndex_copy.x, gridIndex_copy.y);
                        std::cout << "🎯 Goal set on GridMap." << std::endl;
                }else if(fusemap.getIsActive()){
                        fusemap.setGoal(gridIndex_copy.x,gridIndex_copy.y);
                        std::cout << "🎯 Goal set on Fusemap." << std::endl;

                }
                 else {
                        std::cout << "⚠️ No active map to set Goal." << std::endl;
                }

                state.goal = gridIndex_copy;
                state.goalActive = true;
            } else {
                std::cout << "⚠️ Goal is already active. Clear it first." << std::endl;
                }
            }

            hide();
        });
    }

    if (clear) {
        clear->onPress([this]() {
            auto& state = getActiveState();

            if (state.startActive) {
                if (livemap.getIsActive())
                    livemap.clearSetPoints(state.start);
                else if (gridmap.getIsActive())
                    gridmap.clearSetPoints(state.start);

                state.startActive = false;
            }

            if (state.goalActive) {
                if (livemap.getIsActive())
                    livemap.clearSetPoints(state.goal);
                else if (gridmap.getIsActive())
                    gridmap.clearSetPoints(state.goal);

                state.goalActive = false;
            }

            std::cout << "🧹 Start and Goal cleared." << std::endl;
            hide();
        });
    }

    if (clearView) {
        clearView->onPress([this]() {
            std::cout << "🧼 Clear View button pressed (no action assigned)." << std::endl;
            hide();
        });
    }
}


