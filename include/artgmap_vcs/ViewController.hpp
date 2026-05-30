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
 * @brief Controls view, zoom, and panning. Also draws grid and axis numeration.
 */
class ViewController {
public:
    /**
     * @brief Constructor.
     * @param win Reference to the SFML render window.
     * @param metersPerCell Size of each grid cell in meters.
     * @param pixelsPerMeter Scale factor to convert meters to pixels.
     * @param view Reference to the SFML view object to control.
     * @param mapSizeCells Number of cells per side in the map.
     */
    ViewController(sf::RenderWindow& win, float metersPerCell, float pixelsPerMeter, sf::View& view, int mapSizeCells);

    /**
     * @brief Handles mouse movement events for panning and zooming.
     * @param event The SFML event to process.
     */
    void handleEvent(const sf::Event& event);

    /**
     * @brief Applies the current view settings to the window.
     */
    void applyView();

    /**
     * @brief Resets the view to its default state.
     */
    void reset();

    /**
     * @brief Handles zoom in/out based on user input events.
     * @param event The SFML event containing zoom information.
     */
    void zoomController(const sf::Event& event);

    /**
     * @brief Draws the grid lines on the provided render target.
     * @param target SFML render target (e.g., window).
     */
    void drawGrid(sf::RenderTarget& target);

    /**
     * @brief Draws the axes lines and numeration on the render target.
     * @param target SFML render target.
     */
    void drawAxes(sf::RenderTarget& target);

    // --- Getters ---

    /**
     * @brief Returns the mouse position in window pixel coordinates.
     */
    sf::Vector2i windowMousePosition() const { return pixelPos; }

    /**
     * @brief Returns the size of each grid cell in meters.
     */
    float getMetersPerCell() const { return metersPerCell; }

    /**
     * @brief Returns the pixels per meter scale factor.
     */
    float getPixelsPerMeter() const { return pixelsPerMeter; }

    /**
     * @brief Returns mouse position in grid coordinates.
     */
    sf::Vector2i getMousePixelPosition() const { return mousePosition_G; }

    /**
     * @brief Returns mouse position in world coordinates.
     */
    sf::Vector2f getMouseWorldPosition() const { return mousePosition_W; }

    /**
     * @brief Returns the default view of the window.
     */
    sf::View getDefaultView() const { return defaultView; }

    /**
     * @brief Returns the total number of grid cells per side.
     */
    int getMapSizeCells() const { return MAP_SIZE_CELLS; }

    /**
     * @brief Returns the current view.
     */
    sf::View getView() const;

    /**
     * @brief Returns the current zoom level.
     */
    float getZoom() const;

    /**
     * @brief Returns the grid cell index hovered by the mouse.
     * @param gridSize Size of the grid in cells.
     */
    sf::Vector2i getHoveredCell(int gridSize) const;

    void setDragging(bool state){dragging=state;};

private:
    sf::RenderWindow& window; ///< Reference to the SFML window
    sf::View& view;           ///< Reference to the controlled view
    sf::View defaultView;     ///< Default view settings
    sf::View customDefaultView; ///< Custom default view if needed

    bool dragging = false;    ///< True if currently dragging/panning
    sf::Vector2i dragStart;   ///< Mouse position where dragging started

    // Mouse positions
    sf::Vector2f mousePosition_W;  ///< Mouse position in world coordinates
    sf::Vector2i mousePosition_G;  ///< Mouse position in grid coordinates
    sf::Vector2i pixelPos;         ///< Mouse position in window pixels

    float metersPerCell;            ///< Size of each grid cell in meters
    float pixelsPerMeter;           ///< Conversion factor from meters to pixels
    const int MAP_SIZE_CELLS; ///< Number of cells per side in the map

    sf::Font font;           ///< Font used for axis numbering
    bool fontLoaded = false; ///< Flag indicating font load success

    std::vector<std::vector<sf::Vector2f>> cellTopLeft_; ///< Cached coordinates of top-left corners of cells
    bool cellCoordsValid_ = false;                        ///< Indicates if cached cell coordinates are valid
};
