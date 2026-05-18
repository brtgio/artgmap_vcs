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


#include <fstream>
#include <sstream>
#include <string>
#include <iostream>
#include <iomanip>         // For std::setprecision
#include "Map.hpp"
#include "artgslam_vcs_lidar/GridMap.hpp"
#include "ViewController.hpp"
#include "tinyfiledialogs.h"

/**
 * @class FileManager
 * @brief Manages file input/output operations for grid map visualization.
 * This class handles loading data from files to populate the map,
 * saving coordinate data, and exporting screenshots.
 */
class FileManager
{
private:
    GridMap& map;                 ///< Reference to the grid map object
    Map& lmap;                    ///< Reference to sonar map instance
    std::string loadedFilename;  ///< Path of the last loaded or saved file

public:

    FileManager(GridMap& mapRef,Map& lmapRef);

    void loadDialog();

    void saveDialog();

    void dataLoad(const std::string& filename, std::vector<double>& x, std::vector<double>& y);

    void saveData(const std::string& filename, std::vector<double>& x, std::vector<double>& y);

    void saveScreen(const std::string& filename);
};
