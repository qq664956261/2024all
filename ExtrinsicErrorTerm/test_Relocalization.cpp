#include "ExtrinsicErrorTerm/Relocalization.h"

int main()
{
    Relocalization::Ptr pRelocalization;
    pRelocalization = std::shared_ptr<Relocalization>(new Relocalization());
    std::vector<std::vector<double>> Poses;
    std::deque<std::pair<int, double>> PoseTimeStamp;
    pRelocalization->readPose("./ArmyOdom.txt", Poses, PoseTimeStamp);
    pRelocalization->setPoses(Poses, PoseTimeStamp);
    std::vector<std::vector<double>> SonarWaveDatas;
    std::deque<std::pair<int, double>> SonarWaveTimeStamp;
    pRelocalization->readSonarWaveData("./ArmyUltra.txt", SonarWaveDatas, SonarWaveTimeStamp);
    pRelocalization->setSonarData(SonarWaveDatas, SonarWaveTimeStamp);

    std::vector<std::vector<double>> PosesRelocSource;
    std::deque<std::pair<int, double>> PoseTimeStampRelocSource;
    pRelocalization->readPose("./zhOdom.txt", PosesRelocSource, PoseTimeStampRelocSource);
    pRelocalization->setPosesRelocSource(PosesRelocSource, PoseTimeStampRelocSource);
    std::vector<std::vector<double>> SonarWaveDatasRelocSource;
    std::deque<std::pair<int, double>> SonarWaveTimeStampRelocSource;
    pRelocalization->readSonarWaveData("./zhUltra.txt", SonarWaveDatasRelocSource, SonarWaveTimeStampRelocSource);
    pRelocalization->setSonarDataRelocSource(SonarWaveDatasRelocSource, SonarWaveTimeStampRelocSource);

    pRelocalization->buildMultiFrame();
    pRelocalization->reloc();

    return 0;
}