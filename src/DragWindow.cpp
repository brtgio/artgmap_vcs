#include "artgmap_vcs/DragWindow.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
/**
 * @brief Constructor that initialize DragWindow a window to view the position of robot in real time 
 * @param guiRef Reference to the tgui ui use for menus and windows.
 * @param wmrRef Reference to wmr class.
 */
DragWindow::DragWindow(tgui::Gui &guiRef, UnicicleWmr &wmrRef)
: gui(guiRef), wmr(wmrRef), isDraging(false)
{
        std::string package_path = ament_index_cpp::get_package_share_directory("artgmap_vcs");
    std::string formPath = package_path + "/assets/forms/DragWindow.txt";

    try {
        container = tgui::Group::create();
        container->loadWidgetsFromFile(formPath);
        gui.add(container);

        /// Find the ChildWindow
        panel = container->get<tgui::ChildWindow>("ChildWindow1");

        if (!panel) {
            std::cerr << "❌ 'ChildWindow1' widget not found in form." << std::endl;
            return;
        }

        /// Find the VerticalLayout
        auto layout = panel->get<tgui::VerticalLayout>("VerticalLayout1");
        if (!layout) {
            std::cerr << "❌ 'VerticalLayout1' widget not found in panel." << std::endl;
            return;
        }
        
        /// Get label pointers
        xLabel   = layout->get<tgui::Label>("xLabel");
        yLabel   = layout->get<tgui::Label>("yLabel");
        yawLabel = layout->get<tgui::Label>("yawLabel");
        
        /// Get checkbox pointers
        sonarSel  = panel->get<tgui::CheckBox>("sonarSel");
        cameraSel = panel->get<tgui::CheckBox>("cameraSel");
        fuseSel   = panel->get<tgui::CheckBox>("fuseSel");
        
        if (!xLabel || !yLabel || !yawLabel) {
            std::cerr << "❌ One or more labels not found in layout." << std::endl;
        }
    }
    catch (const tgui::Exception& e) {
        std::cerr << "❌ TGUI Exception while loading PoseWindow: " << e.what() << std::endl;
    }
    
    panel->setKeepInParent(true);
    panel->setPositionLocked(false);
    setupCallbacks(); 
}

/**
 * @brief Sets up actions to do when labels change
 */
void DragWindow::setupCallbacks(){
    if (sonarSel) {
        sonarSel->onChange([this](bool checked){
            if(checked){
                sonarActive=true;
            }
            else{
                sonarActive=false;
            }
            std::cout << "sonarSel changed: " << checked << std::endl;
        });
    }

    if (cameraSel) {
        cameraSel->onChange([this](bool checked){
            if(checked){
                cameraActive=true;
            }
            else{
                cameraActive=false;
            }
            std::cout << "cameraSel changed: " << checked << std::endl;
        });
    }

    if (fuseSel) {
        fuseSel->onChange([this](bool checked){
            if(checked){
                fuseActive=true;
            }
            else{
                fuseActive=false;
            }
            std::cout << "fuseSel changed: " << checked << std::endl;
        });
    }
}
/**
 * @brief Updates robot pose in real time 
 */
void DragWindow::update()
{
    auto [x, y, yaw] = wmr.getPose(); 
    xLabel->setText("X: "   + std::to_string(x));
    yLabel->setText("Y: "   + std::to_string(y));
    yawLabel->setText("Yaw: " + std::to_string(yaw*180.0 / M_PI));
}
/**
 * @brief Sets the window visibility flag 
 */
void DragWindow::show(bool visible){
    if (panel)
        panel->setVisible(visible);
}

/**
 * @brief Handles mouse events for enabling window dragging.
 * 
 * Converts mouse coordinates to TGUI space, checks whether the mouse is over
 * interactive widgets, and manages the dragging state of the panel. Events are
 * passed to TGUI when needed to maintain focus and widget interaction behavior.
 *
 * @param event Reference to the SFML event to process.
 * @param mousePos Current mouse position in window coordinates.
 * @return true If the drag operation is being handled by this function.
 * @return false If the event should be passed to TGUI for handling.
 */
bool DragWindow::handleEvent(const sf::Event& event, const sf::Vector2i& mousePos)
{
    if (!panel) return false;

    // Convert to TGUI coordinates
    tgui::Vector2f guiPos(static_cast<float>(mousePos.x),
                          static_cast<float>(mousePos.y));

    // Check if the mouse is over any interactive widget
    bool mouseOnInteractiveWidget = isMouseOnInteractiveWidget(guiPos);

    if (event.type == sf::Event::MouseButtonPressed &&
        event.mouseButton.button == sf::Mouse::Left)
    {
        // If the click is on an interactive widget, DO NOT handle it here.
        // Let TGUI process it.
        if (mouseOnInteractiveWidget) {
            isDraging = false;
            return false;
        }

        // Check if the click is inside the panel
        if (panel->isMouseOnWidget(guiPos)) {
            isDraging = true;
            dragOffset = guiPos - panel->getPosition();
            
            // IMPORTANT: Send the event to TGUI first so it can manage focus
            // and other internal behavior, but mark that dragging has started.
            gui.handleEvent(event);
            return true; // We are handling the drag
        }
    }
    else if (event.type == sf::Event::MouseButtonReleased &&
             event.mouseButton.button == sf::Mouse::Left)
    {
        if (isDraging) {
            isDraging = false;
            // No need to send the release event to TGUI if we were dragging
            return true;
        }
    }
    else if (event.type == sf::Event::MouseMoved && isDraging)
    {
        // Only drag while in dragging mode
        panel->setPosition(guiPos - dragOffset);
        return true; // Consume movement event during dragging
    }

    // For all other cases, let TGUI handle the event
    return false;
}


/**
 * @brief Checks whether the mouse is currently over any interactive widget.
 * 
 * Iterates through the list of interactive widgets (such as selectors or buttons)
 * and determines if the mouse position intersects with any of them.
 *
 * @param mousePos Mouse position in TGUI coordinate space.
 * @return true If the mouse is over an interactive widget.
 * @return false Otherwise.
 */
bool DragWindow::isMouseOnInteractiveWidget(const tgui::Vector2f& mousePos) const
{
    std::vector<tgui::Widget::Ptr> interactiveWidgets = {
        sonarSel, cameraSel, fuseSel
    };
    
    for (const auto& widget : interactiveWidgets) {
        if (widget && widget->isMouseOnWidget(mousePos)) {
            return true;
        }
    }
    return false;
}
