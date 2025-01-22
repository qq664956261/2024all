#include "ExtrinsicErrorTerm/MapOptimization.h"

int main()
{
    MapOptimization::Ptr pMapOptimization;
    pMapOptimization = std::shared_ptr<MapOptimization>(new MapOptimization());
    pMapOptimization->readPose(".");
    pMapOptimization->readSonarWaveData(".");
    pMapOptimization->findFirstLapEndTime();
    pMapOptimization->optimize();
    return 0;
}