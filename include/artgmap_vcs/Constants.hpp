#pragma once

namespace Constants {
    constexpr float DEFAULT_METERS_PER_CELL = 0.05f;  ///< Default size of each grid cell in meters
    constexpr float DEFAULT_PIXELS_PER_METER = 50.0f; ///< Default scale factor to convert meters to pixels
    constexpr int MAP_SIZE_CELLS = 900;                   ///< Default number of cells per side in the square grid map
    constexpr int WINDOW_WIDTH = 1024;                 ///< Default width of the application window in pixels
    constexpr int WINDOW_HEIGHT = 768;                ///< Default height of the application window in pixels
}