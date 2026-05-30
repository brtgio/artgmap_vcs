#pragma once
#include <iostream>
#include <vector>
#include <cmath>
#include "artgmap_vcs/Map.hpp"

class MapFusion{


    private:
    //Probabilities for log-odds fusion algorithm
        int mapSize;
        float sonarProbOccupied = 0.7f;
        float sonarProbEmpty = 0.3f;

        float cameraProbOccupied = 0.8f;
        float cameraProbEmpty = 0.2f;

        float interProbOccupied = 0.9f;
        float interProbEmpty = 0.1f;


    //Maps to keep track of each cell probability
        std::vector<std::vector<float>> sonarProbMap;
        std::vector<std::vector<float>> cameraProbMap;
        std::vector<std::vector<float>> interProbMap;
        std::vector<std::vector<float>> fuseProbMap;
        std::vector<std::vector<int>> fuseBinMap;


    public:
        MapFusion(int mapSize);
        void map2prob(Map& src,char header);
        void mapFusion();
        const std::vector<std::vector<int>>& getFusedMap() const{
            return fuseBinMap;
        };


};