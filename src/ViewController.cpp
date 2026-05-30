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

#include "artgmap_vcs/ViewController.hpp"
#include <cmath>
#include <iostream>
#include "ament_index_cpp/get_package_share_directory.hpp"

/**
 * @brief Constructor
 * 
 * Initializes the view controller with references to the SFML window and view,
 * as well as parameters defining grid cell size and pixel scale.
 * Loads a custom font for rendering axis labels.
 * 
 * @param win Reference to the SFML render window.
 * @param metersPerCell_ Size of each grid cell in meters.
 * @param pixelsPerMeter_ Scale factor from meters to pixels.
 * @param view Reference to the SFML view to control.
 * @param mapSizeCells Number of cells per side in the map.
 */
ViewController::ViewController(sf::RenderWindow& win, float metersPerCell_, float pixelsPerMeter_, sf::View& view, int mapSizeCells)
    : window(win), metersPerCell(metersPerCell_), pixelsPerMeter(pixelsPerMeter_), view(view), MAP_SIZE_CELLS(mapSizeCells)
{
    /** Initialize the view with a zoom level of 3 (closer zoom) */
    defaultView = window.getDefaultView();

    view = defaultView;
    view.setCenter(0.f, 0.f);
    view.setSize(defaultView.getSize() / 3.f);
    customDefaultView = view;

    
    /** Load font from ROS package assets for axis labels */
    std::string package_path = ament_index_cpp::get_package_share_directory("artgmap_vcs");
    std::string fontPath = package_path + "/assets/fonts/NotoSansMath-Regular.ttf";

    fontLoaded = font.loadFromFile(fontPath);
    if (!fontLoaded) {
        std::cerr << "❌ Error: Could not load font in ViewController from: " << fontPath << std::endl;
    } else {
        std::cout << "✅ Font successfully loaded from: " << fontPath << std::endl;
    }
}

/**
 * @brief Handles all relevant SFML events.
 * 
 * Processes mouse wheel zoom, mouse dragging for panning,
 * Escape key for resetting view, and mouse move events to update positions.
 * 
 * @param event Reference to the SFML event to process.
 */
void ViewController::handleEvent(const sf::Event& event) {
    if (event.type == sf::Event::MouseWheelScrolled) {
        zoomController(event);
    }
    else if (event.type == sf::Event::MouseButtonPressed && event.mouseButton.button == sf::Mouse::Left) {
        dragging = true;
        dragStart = sf::Mouse::getPosition(window);
    }
    else if (event.type == sf::Event::MouseButtonReleased && event.mouseButton.button == sf::Mouse::Left) {
        dragging = false;
    }
    else if (event.type == sf::Event::MouseMoved && dragging) {
        /** Calculate delta movement in world coordinates and move the view accordingly */
        sf::Vector2i now = sf::Mouse::getPosition(window);
        sf::Vector2f delta = window.mapPixelToCoords(dragStart) - window.mapPixelToCoords(now);
        view.move(delta);
        dragStart = now;
    }
    else if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) {
        reset();
    }
    else if (event.type == sf::Event::MouseMoved) {
        /** Update mouse position in pixels and corresponding world coordinates */
        pixelPos = sf::Mouse::getPosition(window);
        mousePosition_W = window.mapPixelToCoords(pixelPos, view);
        mousePosition_G = pixelPos;
    }
}

/**
 * @brief Applies the current view to the SFML window.
 * 
 * Should be called before rendering to set the view properly.
 */
void ViewController::applyView() {
    window.setView(view);
}

/**
 * @brief Resets the view to the custom default view.
 * 
 * This restores the initial zoom and center set in the constructor.
 */
void ViewController::reset() {
    view = customDefaultView;
}

/**
 * @brief Gets the current SFML view object.
 * 
 * @return The current sf::View being used.
 */
sf::View ViewController::getView() const {
    return view;
}

/**
 * @brief Gets the current zoom factor.
 * 
 * The zoom factor is the ratio between the current view size and the default view size.
 * 
 * @return Current zoom factor as a float.
 */
float ViewController::getZoom() const {
    return view.getSize().x / defaultView.getSize().x;
}

/**
 * @brief Calculates the grid cell indices currently hovered by the mouse.
 * 
 * Converts the mouse position from world coordinates to grid indices,
 * taking into account the size of each cell and the grid origin offset.
 * Returns {-1, -1} if the position is outside the grid.
 * 
 * @param gridSize The size (number of cells) of the square grid.
 * @return sf::Vector2i The column and row indices of the hovered cell, or {-1, -1} if invalid.
 */
sf::Vector2i ViewController::getHoveredCell(int gridSize) const
{
    /** Convert mouse world coordinates from pixels to meters */
    float worldX = mousePosition_W.x / pixelsPerMeter;
    float worldY = mousePosition_W.y / pixelsPerMeter;

    /** Adjust for grid origin being centered (offset) */
    float offset = (gridSize * metersPerCell) / 2.0f;

    /** Compute cell indices */
    int col = static_cast<int>(std::floor((worldX + offset) / metersPerCell));
    int row = static_cast<int>(std::floor((worldY + offset) / metersPerCell));

    /** Validate indices are within grid bounds */
    if (col < 0 || col >= gridSize || row < 0 || row >= gridSize)
        return {-1, -1};

    return {col, row};
}

/**
 * @brief Draws the grid lines centered around (0,0).
 * 
 * The grid spans from -mapSizeCells/2 to +mapSizeCells/2 in both directions.
 * Lines are drawn in gray.
 * 
 * @param target The render target on which to draw the grid.
 */
void ViewController::drawGrid(sf::RenderTarget& target) {
    float zoom = defaultView.getSize().x / view.getSize().x;
    float cellSize = metersPerCell * pixelsPerMeter;
    float halfWidth = (MAP_SIZE_CELLS / 2) * cellSize;
    sf::VertexArray lines(sf::Lines);

    int lineCount = 0;

    for (int i = -MAP_SIZE_CELLS / 2; i <= MAP_SIZE_CELLS / 2; ++i) {
        float pos = i * cellSize;

        /** Vertical line */
        lines.append(sf::Vertex({pos, -halfWidth}, sf::Color(100, 100, 100)));
        lines.append(sf::Vertex({pos, halfWidth}, sf::Color(100, 100, 100)));
        lineCount++;

        /** Horizontal line */
        lines.append(sf::Vertex({-halfWidth, pos}, sf::Color(100, 100, 100)));
        lines.append(sf::Vertex({halfWidth, pos}, sf::Color(100, 100, 100)));
        lineCount++;
    }

    target.draw(lines);

    // Debug print (throttled)
    static int frameCounter = 0;
    frameCounter++;

    
}

/**
 * @brief Draws the X and Y axes centered at (0,0) with labels.
 * 
 * X axis is red, Y axis is blue.
 * Labels are drawn every 5 cells along each axis,
 * scaled according to current zoom level for readability.
 * 
 * @param target The render target on which to draw the axes.
 */
void ViewController::drawAxes(sf::RenderTarget& target)
{
    if (!fontLoaded) return;

    /** Save current view */
    sf::View originalView = target.getView();
    target.setView(view);  /** Use current zoomed/panned view */

    sf::VertexArray axes(sf::Lines, 4);
    sf::Vector2f center = view.getCenter();
    sf::Vector2f size = view.getSize();

    /** Calculate visible boundaries */
    float left = center.x - size.x * 0.5f;
    float right = center.x + size.x * 0.5f;
    float top = center.y - size.y * 0.5f;
    float bottom = center.y + size.y * 0.5f;

    /** Draw X axis (red) and Y axis (blue) */
    axes[0] = sf::Vertex({left, 0.f}, sf::Color::Red);
    axes[1] = sf::Vertex({right, 0.f}, sf::Color::Red);
    axes[2] = sf::Vertex({0.f, top}, sf::Color::Blue);
    axes[3] = sf::Vertex({0.f, bottom}, sf::Color::Blue);

    target.draw(axes);

    /** Draw axis labels on the HUD (default view) */
    target.setView(defaultView);

    const float cellSizeWorld = metersPerCell * pixelsPerMeter;   /** Cell size in pixels */
    const float zoomLevel = defaultView.getSize().x / view.getSize().x;

    /** Clamp text scale between 0.5 and 1.5 for readability */
    float textScale = std::clamp(zoomLevel, 0.5f, 1.5f);
    const unsigned baseFontSize = 12;

    /** Calculate visible cells in current view */
    sf::Vector2f topLeft = window.mapPixelToCoords({0, 0}, view);
    sf::Vector2f bottomRight = window.mapPixelToCoords(
        {static_cast<int>(window.getSize().x), static_cast<int>(window.getSize().y)}, view);

    int firstCellX = static_cast<int>(std::floor(topLeft.x / cellSizeWorld));
    int firstCellY = static_cast<int>(std::floor(topLeft.y / cellSizeWorld));
    int numCols = static_cast<int>((bottomRight.x - topLeft.x) / cellSizeWorld) + 2;
    int numRows = static_cast<int>((bottomRight.y - topLeft.y) / cellSizeWorld) + 2;

    /** Draw labels on X axis every 5 cells */
    for (int i = 0; i <= numCols; ++i) {
        int cellX = firstCellX + i;
        if (cellX % 5 != 0) continue;

        float worldX = cellX * cellSizeWorld;
        sf::Vector2i scr = window.mapCoordsToPixel({worldX, 0.f}, view);
        sf::Vector2f pos(static_cast<float>(scr.x) + 2.f, 4.f);

        sf::Text txt(std::to_string(cellX), font, baseFontSize);
        txt.setFillColor(sf::Color::Yellow);
        txt.setScale(textScale, textScale);
        txt.setPosition(pos);

        sf::FloatRect b = txt.getLocalBounds();
        sf::RectangleShape bg({b.width * textScale, b.height * textScale});
        bg.setPosition(pos);
        bg.setFillColor(sf::Color(0, 0, 0, 180));

        target.draw(bg);
        target.draw(txt);
    }

    /** Draw labels on Y axis every 5 cells */
    for (int i = 0; i <= numRows; ++i) {
        int cellY = firstCellY + i;
        if (cellY % 5 != 0) continue;

        float worldY = cellY * cellSizeWorld;
        sf::Vector2i scr = window.mapCoordsToPixel({0.f, worldY}, view);
        sf::Vector2f pos(4.f, static_cast<float>(scr.y) + 2.f);

        sf::Text txt(std::to_string(cellY), font, baseFontSize);
        txt.setFillColor(sf::Color::Yellow);
        txt.setScale(textScale, textScale);
        txt.setPosition(pos);

        sf::FloatRect b = txt.getLocalBounds();
        sf::RectangleShape bg({b.width * textScale, b.height * textScale});
        bg.setPosition(pos);
        bg.setFillColor(sf::Color(0, 0, 0, 180));

        target.draw(bg);
        target.draw(txt);
    }

    /** Restore original view */
    target.setView(originalView);
}

/**
 * @brief Controls zooming based on mouse wheel scroll events.
 * 
 * Limits zoom to be within min and max zoom levels to avoid excessive zooming in or out.
 * 
 * @param event The SFML mouse wheel scroll event triggering zoom.
 */
void ViewController::zoomController(const sf::Event& event) {
    float factor = (event.mouseWheelScroll.delta > 0) ? (1.f / 1.1f) : 1.1f;
    float currentZoom = getZoom();
    float newZoom = currentZoom * factor;

    const float minZoom = 0.1f;
    const float maxZoom = 1.0f;

    //std::cout << "newZoom: " << newZoom << std::endl;

    if (newZoom < minZoom || newZoom > maxZoom) {
        //std::cout << "Zoom out of range [" << minZoom << ", " << maxZoom << "], operation cancelled\n";
        return;
    }

    view.zoom(factor);
}
