#include "artgmap_vcs/MapFusion.hpp"

MapFusion::MapFusion(int size)
{
    this->mapSize = size;
    sonarProbMap.resize(mapSize, std::vector<float>(mapSize, 0.0f));
    cameraProbMap.resize(mapSize, std::vector<float>(mapSize, 0.0f));
    interProbMap.resize(mapSize, std::vector<float>(mapSize, 0.0f));
    fuseProbMap.resize(mapSize, std::vector<float>(mapSize, 0.0f));

    // Inicializar el mapa binario con 0
    fuseBinMap.resize(mapSize, std::vector<int>(mapSize, 0));

}

void MapFusion::map2prob(Map &srcMap, char header)
{
    const auto& grid = srcMap.getGrid(); // grid del Map
    float probOccupie = 0;
    float probEmpty = 0;

    // Selecciona el vector de probabilidades correspondiente
    std::vector<std::vector<float>>* selectedMap = nullptr;
    switch (header) {
        case 's':
            selectedMap = &sonarProbMap;
            probOccupie = sonarProbOccupied;
            probEmpty = sonarProbEmpty;
            break;
        case 'c':
            selectedMap = &cameraProbMap;
            probOccupie = cameraProbOccupied;
            probEmpty = cameraProbEmpty;
            break;
        case 'i':
            selectedMap = &interProbMap;
            probOccupie = interProbOccupied;
            probEmpty = interProbEmpty;
            break;
        default:
            std::cout << "Header doesn't match\n";
            return;
    }

    if (!selectedMap) return;

    // Llenar el mapa de probabilidades según valores del grid
    for (int i = 0; i < mapSize; i++) {
        for (int j = 0; j < mapSize; j++) {
            if (grid[i][j] == 1) {
                (*selectedMap)[i][j] = log(probOccupie / probEmpty); // ejemplo: obstáculo
            } else {
                (*selectedMap)[i][j] = log(probEmpty / probOccupie); // libre
            }
        }
    }
}

void MapFusion::mapFusion()
{
    if (sonarProbMap.empty() || cameraProbMap.empty() || interProbMap.empty()) {
        std::cerr << "[MapFusion] Mapas vacíos, no se puede fusionar.\n";
        return;
    }

    for (int i = 0; i < mapSize; i++) {
        for (int j = 0; j < mapSize; j++) {
            float logSum = sonarProbMap[i][j] + cameraProbMap[i][j] + interProbMap[i][j];
            float p = 1.0f / (1.0f + exp(-logSum));  // convierte a probabilidad real
            fuseProbMap[i][j] = p;
            fuseBinMap[i][j]  = (p > 0.5f) ? 1 : 0;
        }
    }
}
