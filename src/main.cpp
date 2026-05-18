/* * -----------------------------------------------------------------------------
 * Copyright (c) 2026 Gilberto Ramos Valenzuela
 * Artgslam Visualizer Project
 * -----------------------------------------------------------------------------
 */

#include <csignal>
#include <rclcpp/rclcpp.hpp>
#include <SFML/Graphics.hpp>
#include "artgslam_vcs_lidar/MapViewer.hpp"
#include "artgslam_vcs_lidar/Constants.hpp"

static volatile sig_atomic_t g_shutdown_requested = 0;

void handleSignal(int signal) {
    g_shutdown_requested = signal;
}

int main(int argc, char** argv)
{
    // 1. Initialize ROS 2
    rclcpp::InitOptions init_options;
    rclcpp::init(argc, argv, init_options, rclcpp::SignalHandlerOptions::None);

    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    {
        // 2. Create SFML Window
        sf::RenderWindow window(
            sf::VideoMode(Constants::WINDOW_WIDTH, Constants::WINDOW_HEIGHT), 
            "ARTG Map Visualizer"
        );
        window.setFramerateLimit(60); 

        // 3. Initialize MapViewer
        MapViewer mapViewer(window);

        // 4. Main Application Loop
        while (rclcpp::ok() && window.isOpen() && !g_shutdown_requested)
        {
            mapViewer.processEvent();
            mapViewer.update();
            mapViewer.render();

            // 5. Spin the node using the new public getter
            // This pulls the base interface from the private rosconnect object
            rclcpp::spin_some(mapViewer.getRosNode());
        }
    }

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }

    return 0;
}