#pragma once

#include <SFML/Graphics.hpp>
#include <TGUI/Backend/SFML-Graphics.hpp>
#include <TGUI/AllWidgets.hpp>
#include "artgmap_vcs/UnicicleWmr.hpp"

class DragWindow{

    private:

        tgui::Gui& gui;     ///< GUI reference (owned externally)
        UnicicleWmr& wmr;   ///< WMR reference (owned externally)

        tgui::Group::Ptr container; 
        tgui::ChildWindow::Ptr panel;        ///< Panel that acts as the context menu window
        tgui::Label::Ptr xLabel;
        tgui::Label::Ptr yLabel;
        tgui::Label::Ptr yawLabel;
        tgui::Label::Ptr sonarL;
        tgui::Label::Ptr cameraL;
        tgui::Label::Ptr fuseL;
        tgui::CheckBox::Ptr sonarSel;
        tgui::CheckBox::Ptr cameraSel;
        tgui::CheckBox::Ptr fuseSel;

        bool isDraging=0;
        bool sonarActive=0;
        bool cameraActive=0;
        bool fuseActive=0;
        bool lidarActive=0;
        tgui::Vector2f dragOffset;

    public:
        DragWindow(tgui::Gui& guiRef, UnicicleWmr& wmrRef);
        void update();
        void setupCallbacks();
        void show(bool visible);
        bool handleEvent(const sf::Event& event, const sf::Vector2i& mousePos);
        bool getisDragging() const {return isDraging;};
        bool isMouseOnInteractiveWidget(const tgui::Vector2f& mousePos) const; // Agregar esta línea
        bool getSonarActive(){return sonarActive;};
        bool getCameraActive(){return cameraActive;};
        bool getFuseActive(){return fuseActive;};
        bool getLidarActive(){return lidarActive;};
};