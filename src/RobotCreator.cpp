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

#include "artgmap_vcs/RobotCreator.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iostream>
#include <regex>

/**
 * @brief Constructor for RobotCreator.
 * 
 * Loads the GUI form, initializes the window and widgets, and sets up callbacks.
 * 
 * @param wmrRef Reference to the UnicicleWmr robot model to configure.
 */
RobotCreator::RobotCreator(UnicicleWmr& wmrRef)
    : windowRobotCreator(sf::VideoMode(400, 250), "Robot Creator"),
      gui(windowRobotCreator), wmr(wmrRef)
{
    // Load form file path from package share directory
    std::string package_path = ament_index_cpp::get_package_share_directory("artgmap_vcs");
    std::string formPath = package_path + "/assets/forms/createRobot.txt";

    try {
        gui.loadWidgetsFromFile(formPath);
    } catch (const tgui::Exception& e) {
        std::cerr << "Error loading GUI form: " << e.what() << std::endl;
    }

    setupWidgets();   /**< Link widget variables to GUI elements */
    setupCallbacks(); /**< Setup button callbacks */
}

/**
 * @brief Runs the main loop of the Robot Creator window.
 * 
 * Processes events, updates state, and renders GUI until the window is closed.
 */
void RobotCreator::run()
{
    while (isRunning())
    {
        processEvents(); /**< Handle input and window events */
        update();        /**< Update logic (currently unused) */
        render();        /**< Render the GUI */
    }
}

/**
 * @brief Processes all window events.
 * 
 * Polls and handles SFML window events and delegates event handling to TGUI.
 */
void RobotCreator::processEvents()
{
    sf::Event event;
    while (windowRobotCreator.pollEvent(event))
    {
        gui.handleEvent(event);

        if (event.type == sf::Event::Closed)
            windowRobotCreator.close();
    }
}

/**
 * @brief Placeholder for future update logic.
 */
void RobotCreator::update()
{
    // Currently no update logic
}

/**
 * @brief Clears the window and draws the GUI.
 */
void RobotCreator::render()
{
    windowRobotCreator.clear(sf::Color::White);
    gui.draw();
    windowRobotCreator.display();
}

/**
 * @brief Retrieves and validates GUI widgets from the loaded form.
 */
void RobotCreator::setupWidgets()
{
    widthBox  = gui.get<tgui::EditBox>("widthBox");
    heightBox = gui.get<tgui::EditBox>("heightBox");
    colorBox  = gui.get<tgui::EditBox>("colorBox");

    resetButton  = gui.get<tgui::Button>("Button1");
    createButton = gui.get<tgui::Button>("Button2");

    if (!widthBox || !heightBox || !colorBox || !resetButton || !createButton)
    {
        std::cerr << "[ERROR] One or more widgets could not be found.\n";
        return;
    }

    // Optional: set input validators (commented out)
    // widthBox->setInputValidator(R"(^-?\d*\.?\d+$)");
    // heightBox->setInputValidator(R"(^-?\d*\.?\d+$)");
}

/**
 * @brief Sets up callbacks for reset and create buttons.
 * 
 * - Resets all input fields on reset button press.
 * - Validates input and creates robot on create button press.
 */
void RobotCreator::setupCallbacks()
{
    if (!resetButton || !createButton) return;

    resetButton->onPress([this]() {
        widthBox->setText("");
        heightBox->setText("");
        colorBox->setText("");
    });

    createButton->onPress([this]() {
        std::string widthStr  = widthBox->getText().toStdString();
        std::string heightStr = heightBox->getText().toStdString();
        std::string colorStr  = colorBox->getText().toStdString();

        bool valid = true;

        if (!isValidFloat(widthStr)) {
            widthBox->setText("");
            valid = false;
        }

        if (!isValidFloat(heightStr)) {
            heightBox->setText("");
            valid = false;
        }

        if (!isValidHexColor(colorStr)) {
            colorBox->setText("");
            valid = false;
        }

        if (!valid) {
            std::cout << "Error: Invalid input. Please check the fields.\n";
            return;
        }

        float width  = std::stof(widthStr);
        float height = std::stof(heightStr);
        sf::Color color = hexToColor(colorStr);

        wmr.setDimensions(width, height);
        wmr.setColor(color);
        wmr.setRobotActive(true);

        std::cout << "Robot created:\n";
        std::cout << "Width: " << width << ", Height: " << height << "\n";
        std::cout << "Color RGB: (" 
                  << static_cast<int>(color.r) << ", "
                  << static_cast<int>(color.g) << ", "
                  << static_cast<int>(color.b) << ")\n";

        windowRobotCreator.close();
    });
}

/**
 * @brief Checks if the window is still open.
 * @return true if the window is open, false otherwise.
 */
bool RobotCreator::isRunning() const
{
    return windowRobotCreator.isOpen();
}

/**
 * @brief Validates if a string represents a valid floating point number.
 * 
 * @param str The string to validate.
 * @return true if valid float format, false otherwise.
 */
bool RobotCreator::isValidFloat(const std::string& str)
{
    if (str.empty()) return false;
    static const std::regex floatRegex(R"(^-?\d+(\.\d+)?$)");
    return std::regex_match(str, floatRegex);
}

/**
 * @brief Validates if a string is a valid hex color (#RGB or #RRGGBB).
 * 
 * @param str The hex color string.
 * @return true if valid hex color format, false otherwise.
 */
bool RobotCreator::isValidHexColor(const std::string& str)
{
    static const std::regex hexShort(R"(#([A-Fa-f0-9]{3}))");
    static const std::regex hexLong(R"(#([A-Fa-f0-9]{6}))");
    return std::regex_match(str, hexShort) || std::regex_match(str, hexLong);
}

/**
 * @brief Converts a hex color string to an SFML color object.
 * 
 * Supports both 3-digit (#abc) and 6-digit (#aabbcc) hex formats.
 * 
 * @param hex The hex color string.
 * @return Corresponding sf::Color object.
 */
sf::Color RobotCreator::hexToColor(const std::string& hex)
{
    std::string h = hex;
    if (h.empty()) return sf::Color::White;
    if (h[0] == '#') h = h.substr(1);

    if (h.length() == 3)
    {
        h = {h[0], h[0], h[1], h[1], h[2], h[2]};
    }

    unsigned int r = std::stoul(h.substr(0, 2), nullptr, 16);
    unsigned int g = std::stoul(h.substr(2, 2), nullptr, 16);
    unsigned int b = std::stoul(h.substr(4, 2), nullptr, 16);

    return sf::Color(r, g, b);
}

