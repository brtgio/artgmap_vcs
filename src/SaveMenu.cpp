#include "artgslam_vcs_lidar/SaveMenu.hpp"

/**
 * @brief Constructs the SaveMenu window and loads GUI components.
 *
 * Loads the GUI form from the ROS package, initializes all widgets,
 * and configures button callbacks for saving map and robot data.
 *
 * @param livemap   Reference to the live (sonar) map.
 * @param kinnectmap Reference to the depth camera map.
 * @param fusionmap  Reference to the fusion map.
 * @param wmr        Reference to the robot (WMR) object to retrieve pose.
 */
SaveMenu::SaveMenu(Map &livemap, Map &kinnectmap, Map &fusionmap, UnicicleWmr &wmr)
: liveMap(livemap), kinectMap(kinnectmap), fusionMap(fusionmap), wmr(wmr), gui(saveWindow), saveWindow(sf::VideoMode(300, 250), "Save Menu")
{
    /// Load form file path from the ROS package
    std::string package_path = ament_index_cpp::get_package_share_directory("artgslam_vcs_lidar");
    std::string formPath = package_path + "/assets/forms/saveMenu.txt";

    try {
        gui.loadWidgetsFromFile(formPath);
    } catch (const tgui::Exception& e) {
        std::cerr << "Error loading GUI form: " << e.what() << std::endl;
    }

    setupWidgets();   /**< Links widget variables to GUI elements */
    setupCallbacks(); /**< Sets up button callbacks */
}

/**
 * @brief Runs the Save Menu event loop.
 *
 * Processes window events, updates logic (if needed),
 * and renders the GUI until the window is closed.
 */
void SaveMenu::run()
{
    while (saveWindow.isOpen()){
        processEvents(); /**< Handle input and window events */
        update();        /**< Update logic (currently unused) */
        render();        /**< Render the GUI */
    }
}

/**
 * @brief Logic update function (currently unused).
 */
void SaveMenu::update()
{
}

/**
 * @brief Processes SFML window events and forwards them to TGUI.
 */
void SaveMenu::processEvents()
{
    sf::Event event;
    while (saveWindow.pollEvent(event)) {
        if (event.type == sf::Event::Closed)
            saveWindow.close();

        gui.handleEvent(event); ///< Forward events to TGUI
    }
}

/**
 * @brief Clears and redraws the GUI window.
 */
void SaveMenu::render()
{
    saveWindow.clear(sf::Color(245, 245, 245));
    gui.draw();
    saveWindow.display();
}

/**
 * @brief Retrieves widget pointers from the loaded GUI form.
 *
 * Connects all GUI components (buttons, checkboxes)
 * to internal class variables for later use.
 */
void SaveMenu::setupWidgets()
{
    saveMultipleButton = gui.get<tgui::Button>("SaveA");
    savePButton        = gui.get<tgui::Button>("saveP");
    saveSButton        = gui.get<tgui::Button>("saveS");
    saveDCButton       = gui.get<tgui::Button>("saveDC");
    saveFButton        = gui.get<tgui::Button>("saveF");

    cbWMR    = gui.get<tgui::CheckBox>("CheckBox1");
    cbSonar  = gui.get<tgui::CheckBox>("CheckBox2");
    cbDC     = gui.get<tgui::CheckBox>("CheckBox3");
    cbFusion = gui.get<tgui::CheckBox>("CheckBox4");
}

/**
 * @brief Sets up callback functions for all save buttons.
 *
 * Each button opens a file dialog and saves:
 * - robot pose
 * - sonar map
 * - depth camera map
 * - fusion map
 * - or a combined CSV file with selected fields
 */
void SaveMenu::setupCallbacks()
{
    saveMultipleButton->onPress([this](){
        /// Open save dialog for CSV output
        const char* filePathC = tinyfd_saveFileDialog(
            "Guardar CSV",
            "data.csv",
            0, nullptr, nullptr
        );
        if (!filePathC) return; ///< User cancelled
        std::string filePath(filePathC);

        std::ofstream file(filePath);
        if(!file.is_open()){
            tinyfd_messageBox("Error","No se pudo abrir el archivo para escribir", "ok", "error", 1);
            return;
        }

        /// Dynamic header generation based on selected checkboxes
        std::vector<std::string> headers;
        if(cbWMR->isChecked())    headers.insert(headers.end(), {"X","Y","Theta"});
        if(cbSonar->isChecked())  headers.insert(headers.end(), {"SonarX","SonarY"});
        if(cbDC->isChecked())     headers.insert(headers.end(), {"DCX","DCY"});
        if(cbFusion->isChecked()) headers.insert(headers.end(), {"FusionX","FusionY"});

        for(size_t i=0;i<headers.size();++i){
            file << headers[i];
            if(i < headers.size()-1) file << ",";
        }
        file << "\n";

        /// Determine maximum number of rows to write
        size_t maxRows = 0;
        if(cbSonar->isChecked())  maxRows = std::max(maxRows, liveMap.getPoints().size());
        if(cbDC->isChecked())     maxRows = std::max(maxRows, kinectMap.getPoints().size());
        if(cbFusion->isChecked()) maxRows = std::max(maxRows, fusionMap.getPoints().size());
        if(cbWMR->isChecked())    maxRows = std::max(maxRows, size_t(1));

        /// Cache point vectors
        auto sonarPoints   = liveMap.getPoints();
        auto dcPoints      = kinectMap.getPoints();
        auto fusionPoints  = fusionMap.getPoints();

        /// Write all rows
        for(size_t i=0;i<maxRows;++i){
            bool first = true;

            if(cbWMR->isChecked()){
                auto [x,y,t] = wmr.getPose();
                if(!first) file << ",";
                file << x << "," << y << "," << t;
                first = false;
            }

            if(cbSonar->isChecked()){
                if(i < sonarPoints.size()){
                    auto [x,y] = sonarPoints[i];
                    if(!first) file << ",";
                    file << x << "," << y;
                    first = false;
                } else if(!first) file << ",,";
            }

            if(cbDC->isChecked()){
                if(i < dcPoints.size()){
                    auto [x,y] = dcPoints[i];
                    if(!first) file << ",";
                    file << x << "," << y;
                    first = false;
                } else if(!first) file << ",,";
            }

            if(cbFusion->isChecked()){
                if(i < fusionPoints.size()){
                    auto [x,y] = fusionPoints[i];
                    if(!first) file << ",";
                    file << x << "," << y;
                } else if(!first) file << ",,";
            }

            file << "\n";
        }

        file.close();
        tinyfd_messageBox("Éxito", "Archivo CSV guardado correctamente", "ok", "info", 1);
    });

    /// Save robot pose
    savePButton->onPress([this](){
        const char* filePathC = tinyfd_saveFileDialog(
            "Guardar Pose del Robot",
            "pose.csv",
            0, nullptr, nullptr
        );
        if(!filePathC) return;
        std::ofstream file(filePathC);
        if(!file.is_open()){
            tinyfd_messageBox("Error", "No se pudo abrir el archivo para escribir", "ok", "error", 1);
            return;
        }

        file << "X,Y,Theta\n";
        auto [x,y,t] = wmr.getPose();
        file << x << "," << y << "," << t << "\n";
        file.close();
        tinyfd_messageBox("Éxito", "Pose guardada correctamente", "ok", "info", 1);
    });

    /// Save sonar map
    saveSButton->onPress([this](){
        const char* filePathC = tinyfd_saveFileDialog(
            "Guardar Sonar Map",
            "sonar_map.csv",
            0, nullptr, nullptr
        );
        if(!filePathC) return;
        std::ofstream file(filePathC);
        if(!file.is_open()){
            tinyfd_messageBox("Error", "No se pudo abrir el archivo para escribir", "ok", "error", 1);
            return;
        }

        const auto& xs = liveMap.getRealX();
        const auto& ys = liveMap.getRealY();
        if(xs.size()!=ys.size()){
            tinyfd_messageBox("Error", "Vectores de coordenadas distintos tamaños", "ok", "error", 1);
            return;
        }

        file << "SonarX,SonarY\n";
        for(size_t i=0;i<xs.size();++i)
            file << xs[i] << "," << ys[i] << "\n";

        file.close();
        tinyfd_messageBox("Éxito", "Sonar map guardado correctamente", "ok", "info", 1);
    });

    /// Save depth camera map
    saveDCButton->onPress([this](){
        const char* filePathC = tinyfd_saveFileDialog(
            "Guardar Deep Camera Map",
            "dc_map.csv",
            0, nullptr, nullptr
        );
        if(!filePathC) return;
        std::ofstream file(filePathC);
        if(!file.is_open()){
            tinyfd_messageBox("Error", "No se pudo abrir el archivo para escribir", "ok", "error", 1);
            return;
        }

        const auto& xs = kinectMap.getRealX();
        const auto& ys = kinectMap.getRealY();
        if(xs.size()!=ys.size()){
            tinyfd_messageBox("Error", "Vectores de coordenadas distintos tamaños", "ok", "error", 1);
            return;
        }

        file << "DCX,DCY\n";
        for(size_t i=0;i<xs.size();++i)
            file << xs[i] << "," << ys[i] << "\n";

        file.close();
        tinyfd_messageBox("Éxito", "Deep Camera map guardado correctamente", "ok", "info", 1);
    });

    /// Save fusion map
    saveFButton->onPress([this](){
        const char* filePathC = tinyfd_saveFileDialog(
            "Guardar Fusion Map",
            "fusion_map.csv",
            0, nullptr, nullptr
        );
        if(!filePathC) return;
        std::ofstream file(filePathC);
        if(!file.is_open()){
            tinyfd_messageBox("Error", "No se pudo abrir el archivo para escribir", "ok", "error", 1);
            return;
        }

        const auto& xs = fusionMap.getRealX();
        const auto& ys = fusionMap.getRealY();
        if(xs.size()!=ys.size()){
            tinyfd_messageBox("Error", "Vectores de coordenadas distintos tamaños", "ok", "error", 1);
            return;
        }

        file << "FusionX,FusionY\n";
        for(size_t i=0;i<xs.size();++i)
            file << xs[i] << "," << ys[i] << "\n";

        file.close();
        tinyfd_messageBox("Éxito", "Fusion map guardado correctamente", "ok", "info", 1);
    });
}
