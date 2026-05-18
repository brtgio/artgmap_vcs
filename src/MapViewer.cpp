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

#include "artgslam_vcs_lidar/MapViewer.hpp"
#include "artgslam_vcs_lidar/Constants.hpp"
#include <sstream>
#include <iomanip>
#include <algorithm>
using namespace Constants;
/**
 * @brief Constructs a MapViewer instance.
 *
 * Initializes references to the SFML window, GUI system, controllers, maps,
 * menu bar, ROS communication handler, robot model, live mapping systems,
 * right-click context menu, and A* simulation engine.
 * Also registers callback functions for UI events and connects signals.
 *
 * @param win Reference to the SFML RenderWindow where rendering occurs.
 */
bool navigation_state = false;
MapViewer::MapViewer(sf::RenderWindow& win)
    : window(win)                           /// Store reference to SFML window
    , view(window.getDefaultView())        /// Initialize default camera view
    , gui(win)                              /// Initialize GUI with window reference
    , controller(win,DEFAULT_METERS_PER_CELL, DEFAULT_PIXELS_PER_METER, view, MAP_SIZE_CELLS)    /// Initialize ViewController with parameters
    , map(MAP_SIZE_CELLS,DEFAULT_METERS_PER_CELL, controller)            /// Initialize stored map with size, resolution, controller
    , manager(map, livemap)                 /// FileManager with reference to maps
    , menu(gui)                             /// MenuBar with GUI reference
    , rosconnect(aStarsim)                  /// ROS data handler for sensors and navigation
    , wmr()                                  /// Wheeled Mobile Robot model
    , livemap(MAP_SIZE_CELLS, DEFAULT_METERS_PER_CELL, controller)        /// Sonar live map instance
    , kinnectmap(MAP_SIZE_CELLS,DEFAULT_METERS_PER_CELL, controller)     /// Kinect live map instance
    , fusionmap(MAP_SIZE_CELLS, DEFAULT_METERS_PER_CELL, controller)      /// Map fusion instance
    , r_menu(gui, map, livemap, fusionmap)  /// Right-click menu with GUI, map, and live maps
    , aStarsim(map, livemap, fusionmap)     /// A* pathfinding simulator with maps
    , dragwin(gui, wmr)                     /// Drag window for showing robot position
    //, nodectrl(gui)                         /// Node controller menu
{
    r_menu.connectSignals();  /**< Connect right-click menu event callbacks */

    /**
     * @brief Set callback functions for menu bar actions.
     */
    menu.setCallbacks(

        /// Load map CSV file
        [this]() {
            manager.loadDialog();
        },

        /// Open save window for exporting maps
        [this]() {
            SaveMenu save(livemap, kinnectmap, fusionmap, wmr);
            save.run();
        },

        /// TODO: Implement image saving functionality
        [this]() { 

            manager.saveDialog();
        },

        /// Exit the application
        [this]() { 
            running = false; 
            window.close();
        },

        /// Reset the camera view to default
        [this]() { 
            controller.reset();
        },

        /// Clear stored and live map data
        [this]() { 
            map.clearGrid(); 
            livemap.clearGrid();
        },

        /// Open robot configuration window
        [this]() {
            RobotCreator creator(wmr);
            creator.run();
        },

        /// Activate A* pathfinding and start animation
        [this]() {
            rosconnect.resetNavigation();
            aStarsim.updatemap();
            aStarsim.start();
            astarAnimating = true;
        },

        /// Clear A* computed paths
        [this]() {
            aStarsim.clearPath();
        },

        /// Toggle robot model visibility
        [this]() {
            bool newState = !wmr.getRobotActive();
            wmr.setRobotActive(newState);
        },

        /// Execute navigation: publish target pose and send full path
        [this]() {
            const auto& pathCells = aStarsim.getOptimizePath();

            if (pathCells.empty())
            return;

            rosconnect.setExecutePathActivity(false);
             
            rosconnect.resetNavigation();

            auto pathMeters = aStarsim.path2meters(pathCells);

for (size_t i = 0; i < pathMeters.size(); ++i) {
    std::cout << "(" 
              << pathMeters[i].x << ", " 
              << pathMeters[i].y << ")\n";
}
            rosconnect.setPath(pathMeters);
            navigation_state=true;
            rosconnect.setExecutePathActivity(true);
        },

        /// Reset robot pose and clear live map
        [this]() {
            wmr.setPose(0, 0, 0);
            aStarsim.clearPath();
            livemap.clearGrid();
            livemap.clearPoints();
        },

        /// Open node controller panel
        [this]() {
            //nodectrl.show(true);
        },

        /// Fill small gaps in grid map (post-processing)
        [this]() {
            map.possProcessMap();
        },

        /// Perform map fusion (live sonar + kinect)
        [this]() {

            /// Activate fusion map
            fusionmap.setActive(true);

            const auto& gridA = livemap.getGrid();
            const auto& gridB = kinnectmap.getGrid();

            if (gridA.empty() || gridB.empty()) {
                std::cerr << "Cannot fuse: one of the maps is empty.\n";
                return;
            }

            const int rows = std::min(gridA.size(), gridB.size());
            const int cols = std::min(gridA[0].size(), gridB[0].size());

            fusionmap.clearGrid();  /// Clear fusion map before processing

            double res = 0.1;       /// Map resolution
            int centerX = cols / 2;
            int centerY = rows / 2;

            for (int r = 0; r < rows; ++r) {
                for (int c = 0; c < cols; ++c) {

                    int valA = livemap.isOccupied(c, r);
                    int valB = kinnectmap.isOccupied(c, r);

                    int fused = 0;

                    if (valA == 1 && valB == 1) {
                        fused = 1;
                    }
                    else if (valA == 's' && valB == 's') {
                        fused = 's';
                    }
                    else {
                        fused = 0;
                    }

                    fusionmap.setObsGrid(c, r, fused);

                    /// Add fused occupied points to list (in world coordinates)
                    if (fused == 1) {
                        double x = (c - centerX) * res;
                        double y = -(r - centerY) * res;
                        fusionmap.addPoint(x, y);
                    }
                }
            }

            std::cout << "Fusion map created successfully.\n";
        },

        /// TODO: Implement window filter for map fusion
        [this]() {
            /// TODO: Implement windowed filtering for fusion map
        }
    );
}


/**
 * @brief Updates the visualization state, robot pose, live sensor maps,
 *        A* animation and GUI components, each frame.
 */
void MapViewer::update()
{
    int gridSize = map.getMapSize();

    /// Gets the grid cell currently under the mouse cursor.
    gridIndex = controller.getHoveredCell(gridSize);

    /// Prepare coordinate status string with fixed precision
    std::ostringstream oss;
 
    if (gridIndex.x != -1 && gridIndex.y != -1) {
        oss << " | Grid: (" << gridIndex.x << ", " << gridIndex.y << ")";
    } else {
        oss << " | Out of bounds";
    }

    /// Updates the coordinate display in the status bar.
    menu.updateCoordinates(oss.str());


    /// Updates the robot's on-screen position using the latest pose, only if a new pose is available.
    
        const Pose& pose = rosconnect.getPose();
        float x = pose.x;
        float y = pose.y;
        float theta = pose.theta;
        //std::cout << "Received Pose: (" << x << ", " << y << ", " << theta << ")\n";
        wmr.setPose(x, y, theta);

 

        map.setStartOnflyFromReal(pose.x,-pose.y);
        kinnectmap.setStartOnfly(wmrGridPose);
        livemap.setStartOnfly(wmrGridPose);
        fusionmap.setStartOnfly(wmrGridPose);
    
    


    // ----------------------------------------------------------------------
    /// @brief Live Mode: Handles real-time sensor updates and live mapping.
    // ----------------------------------------------------------------------
    if (menu.getLiveMode()) {

        map.setActive(false);

        //livemap.clearPoints();   /**< Clears existing sonar point cloud. */
        //livemap.clearGrid();     /**< Clears live occupancy grid.       */

        kinnectmap.clearPoints(); /**< Clears Kinect point cloud.        */
        kinnectmap.clearGrid();   /**< Clears Kinect occupancy grid.     */

        /// Adds lidar points to the live map.
/// Adds lidar points to the live map transformed to the Fixed Frame.
const auto& lidar = rosconnect.getLidarPoints();
// robot pose is no longer needed for the point-by-point calculation here 
// because p.x and p.y are already in the Fixed Frame (Global).

for (const auto& p : lidar) {
    // The points p.x and p.y now represent global coordinates 
    // calculated via tf2 in the receiver callback.
    float globalX = p.x;
    float globalY = p.y;

    // Add the global point directly to the live map.
    // If your SFML/Visualization logic requires a Y-flip for screen coordinates:
    livemap.addPoint(globalX, globalY); 
}

        /// Adds Kinect points to the Kinect live map.
        const auto& kinnect_p = rosconnect.getKinnectPoints();
        for (const auto& p : kinnect_p) {
            kinnectmap.addPoint(p.x, p.y);
        }

        /// Builds occupancy grids using the updated point clouds.
        livemap.updateGridFromPoints();
        kinnectmap.updateGridFromPoints();
 
    } else {
        /// Disables live mode and restores static map rendering.
        map.setActive(true);
        livemap.setActive(false);
        kinnectmap.setActive(false);
        fusionmap.setActive(false);
    }

    // ----------------------------------------------------------------------
    /// @brief Handles step-by-step A* pathfinding animation.
    // ----------------------------------------------------------------------
    if (astarAnimating) {
        if (!aStarsim.isFinished()) {
            bool continueAnim = aStarsim.step();
            if (!continueAnim) {
                astarAnimating = false;
            }
        } else {
            astarAnimating = false;
        }
    }



    /// Updates drag window/tool state.
    dragwin.update();
}


/**
 * @brief Processes SFML window events.
 *
 * Handles all user interactions including window closing, GUI input,
 * mouse clicks, dragging operations, and context menu behavior.
 */
void MapViewer::processEvent()
{
    sf::Event event;

    while (window.pollEvent(event)) {

        /// DragWindow processes the event first.
        sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        dragwin.handleEvent(event, mousePos);

        /// If the window is being dragged, events are not forwarded to GUI or the controller.
        if (!dragwin.getisDragging()) {
            gui.handleEvent(event);        ///< Standard GUI event handling.
            controller.handleEvent(event); ///< View/controller input handling.
        }
        
        /// Handles window close event.
        if (event.type == sf::Event::Closed) {
            running = false;
            window.close();
        }

        /// Handles mouse clicks for the contextual menu (only if not dragging).
        if (!dragwin.getisDragging() && event.type == sf::Event::MouseButtonPressed) {

            const int gridSize = map.getMapSize();
            const sf::Vector2i cell = controller.getHoveredCell(gridSize);
            const sf::Vector2i pixelPos = sf::Mouse::getPosition(window);
            const sf::Vector2f pixelPosF(pixelPos);

            /// Right-click opens the context menu at cursor position.
            if (event.mouseButton.button == sf::Mouse::Right) {
                r_menu.show(static_cast<float>(pixelPos.x),
                            static_cast<float>(pixelPos.y),
                            cell);
                r_menu.setVisible(true);
            }
            /// Left-click hides the context menu if clicking outside of it.
            else if (event.mouseButton.button == sf::Mouse::Left) {
                if (r_menu.isVisible() && !r_menu.containsPoint(pixelPosF)) {
                    r_menu.setVisible(false);
                }
            }
        }

        /// Handles map cell editing with Shift + Left Click.
        if (event.type == sf::Event::MouseButtonPressed) {

            /// Adds a single obstacle to the main map (if active).
            if (event.mouseButton.button == sf::Mouse::Left &&
                sf::Keyboard::isKeyPressed(sf::Keyboard::LShift) &&
                map.getIsActive()) 
            {
                map.setSingleObs(gridIndex.x, gridIndex.y);
            }

            /// Adds a single obstacle to the live map (if active).
            if (event.mouseButton.button == sf::Mouse::Left &&
                sf::Keyboard::isKeyPressed(sf::Keyboard::LShift)) 
            {
                livemap.setSingleObs(gridIndex.x, gridIndex.y);
            }
        }
    }
}



/**
 * @brief Renders the entire map viewer frame.
 *
 * Clears the window, applies the current view, draws the grid, axes,
 * live or stored maps, A* simulation, robot model, GUI overlays,
 * and finally displays the rendered frame.
 */
void MapViewer::render()
{
    /// Clear window and apply the current view (zoom/pan)
    window.clear(sf::Color::Black);
    controller.applyView();

    /// Draw grid and axis lines
    controller.drawGrid(window);
    controller.drawAxes(window);

    /// Reset map activation flags before updating based on mode
    livemap.setActive(false);
    kinnectmap.setActive(false);
    fusionmap.setActive(false);

    if (menu.getLiveMode())
    {
        /// Update active maps based on DragWindow panel selections
        livemap.setActive(dragwin.getSonarActive());
        kinnectmap.setActive(dragwin.getCameraActive());
        fusionmap.setActive(dragwin.getFuseActive());

        /// Draw only the maps that are active
        if (livemap.getIsActive()) {
            const auto& gridLive = livemap.getGrid();
            if (!(gridLive.empty() || gridLive[0].empty())) {
                livemap.drawLiveMap(window, controller.getPixelsPerMeter(), sf::Color::Yellow);
            } else {
            }
        }

        if (kinnectmap.getIsActive()) {
            const auto& k_grid = kinnectmap.getGrid();
            if (!(k_grid.empty() || k_grid[0].empty())) {
                kinnectmap.drawLiveMap(window, controller.getPixelsPerMeter(), sf::Color::Cyan);
            } else {
            }
        }

        if (fusionmap.getIsActive()) {
            const auto& f_grid = fusionmap.getGrid();
            if (!(f_grid.empty() || f_grid[0].empty())) {
                fusionmap.drawLiveMap(window, controller.getPixelsPerMeter(), sf::Color::Green);
            } else {
            }
        }
    }
    else
    {
        /// Non-live mode → draw the stored map only
        const auto& gridMap = map.getGrid();
        if (!(gridMap.empty() || gridMap[0].empty())) {
            map.draw(window, 50.0f);
        } else {

        }
    }

    /// Draw A* simulation if animation is active
    if (astarAnimating)
        aStarsim.draw(window, 50.0f, controller.getMetersPerCell());

    /// Draw A* resulting paths
    aStarsim.drawFoundPath(window, 50.0f, controller.getMetersPerCell());
    aStarsim.drawSimplifiPath(window, 50.0f, controller.getMetersPerCell());

    /// Draw current robot pose
    wmr.draw(window);

    /// Draw GUI elements (reset to default view first)
    window.setView(window.getDefaultView());
    gui.draw();

    /// Present the final rendered frame
    window.display();
}


/**
 * @brief Checks if the MapViewer application is running.
 * 
 * @return true if the application is running and window is open, false otherwise.
 */
bool MapViewer::isRunning() const
{
    return running && window.isOpen();
}