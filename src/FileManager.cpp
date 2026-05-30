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

#include "artgmap_vcs/FileManager.hpp"

/**
 * @brief Constructor that stores a reference to the GridMap instance for interaction.
 * @param mapRef Reference to the GridMap.
 */
FileManager::FileManager(GridMap& mapRef,Map& lmapRef)
    : map(mapRef),lmap(lmapRef)
{
}

/**
 * @brief Opens a file dialog to select a CSV dataset file, loads the data,
 *        and updates the map accordingly.
 */
void FileManager::loadDialog()
{
    const char* path = tinyfd_openFileDialog(
        "Open Dataset", // Dialog title
        "",             // Default path
        0,              // Number of filters (0 = none)
        nullptr,        // Filter patterns (ignored when count=0)
        nullptr,        // Filter description
        0               // Allow multiple selection? (0 = no)
    );

    if (path) {
        std::cout << "Selected file: " << path << std::endl;
        loadedFilename = path;

        // Load CSV data into vectors
        std::vector<double> xData, yData;
        dataLoad(loadedFilename, xData, yData);

        // Update map points with loaded real-world coordinates
        map.setPoints(xData, yData);

        // Convert real-world coordinates to grid indices and fill the occupancy grid
        std::vector<int> xGrid, yGrid;
        map.xy2Grid(map.getRealX(), map.getRealY(), xGrid, yGrid);
        map.fillGrid(xGrid, yGrid);
    } else {
        std::cout << "No file was selected.\n";
    }
}

/**
 * @brief Opens a save file dialog and saves current real-world map data to a CSV file.
 */
void FileManager::saveDialog()
{
    std::cout<< "save dialog activado"<<std::endl;
    const char* path = tinyfd_saveFileDialog(
        "Save File",       // Dialog title
        "./output.txt",      // Default filename
        0,                 // Number of filters (0 = none)
        nullptr,           // Filter patterns
        nullptr            // Filter description
    );

    if (path) {
        std::cout << "Saving to: " << path << std::endl;

        // Save the real-world coordinates to the file
        if(lmap.getIsActive()==false){

        map.grid2XY(); 
        saveData(path,

                 const_cast<std::vector<double>&>(map.getRealX()),
                 const_cast<std::vector<double>&>(map.getRealY()));
        }
        else{
            saveData(path,
                 const_cast<std::vector<double>&>(lmap.getRealX()),
                 const_cast<std::vector<double>&>(lmap.getRealY()));
        }
    } else {
        std::cout << "Save operation was canceled.\n";
    }
}

/**
 * @brief Loads (x,y) data points from a CSV file into the provided vectors.
 * @param filename Path to the CSV file.
 * @param x Output vector for x coordinates.
 * @param y Output vector for y coordinates.
 */
void FileManager::dataLoad(const std::string& filename, std::vector<double>& x, std::vector<double>& y)
{
    x.clear();
    y.clear();

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return;
    }

    std::string line;
    size_t count = 0;
    bool headerSkipped = false;

    while (std::getline(file, line)) {
        // Saltar líneas vacías o con solo espacios
        if (line.empty() || std::all_of(line.begin(), line.end(), isspace))
            continue;

        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> tokens;

        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }

        if (tokens.size() >= 2) {
            // Detectar encabezado automáticamente
            if (!headerSkipped && !std::isdigit(tokens[0][0]) && tokens[0] != "-" && tokens[0] != ".") {
                headerSkipped = true;
                std::cout << "Header detected and skipped: " << line << "\n";
                continue;
            }

            try {
                double xVal, yVal;

                if (tokens.size() == 2) {
                    xVal = std::stod(tokens[0]);
                    yVal = std::stod(tokens[1]);
                } else {
                    xVal = std::stod(tokens[1]);
                    yVal = std::stod(tokens[2]);
                }

                x.push_back(xVal);
                y.push_back(yVal);
                ++count;
            } catch (const std::exception& e) {
                std::cerr << "Error parsing line: " << line << " (" << e.what() << ")\n";
            }
        } else {
            std::cerr << "Invalid line format (less than 2 columns): " << line << "\n";
        }
    }

    std::cout << "✅ Loaded " << count << " points\n";
}


/**
 * @brief Saves vectors of x and y coordinates to a CSV file.
 * @param filename Path to the output file.
 * @param x Vector of x coordinates.
 * @param y Vector of y coordinates.
 */
void FileManager::saveData(const std::string &filename, std::vector<double> &x, std::vector<double> &y)
{
    if (x.size() != y.size()) {
        std::cerr << "Error: x and y vector sizes do not match.\n";
        return;
    }

    std::ofstream outFile(filename);
    if (!outFile) {
        std::cerr << "Error: Cannot open file for writing.\n";
        return;
    }

    // Use high precision for floating point output
    outFile << std::fixed << std::setprecision(17);
    for (size_t i = 0; i < x.size(); ++i) {
        outFile << x[i] << "," << y[i] << "\n";
    }

    outFile.close();
    std::cout << "Text file saved successfully.\n";
}

/**
 * @brief Placeholder for saving a screenshot of the map window.
 * @param filename Suggested filename for saving.
 * 
 * Note: Implementation to capture and save SFML window content is pending.
 */
void FileManager::saveScreen(const std::string &filename)
{
    const char* path = tinyfd_saveFileDialog(
        "Save Image",
        "Map.png",
        0,
        nullptr,
        nullptr
    );

    if (path) {
        // TODO: Implement screenshot capture and saving using SFML RenderWindow
        return;
    }
}
