#include "PointCloudProcessor.h"

int main(int argc, char **argv)
{

    YAML::Node config = YAML::LoadFile("../config/config-helimos.yaml");

    PointCloudProcessor OusterProcessor(config, 0);
    OusterProcessor.loadTrajectoryAndPoses();
    OusterProcessor.loadAndProcessBinFiles();

    PointCloudProcessor VelodyneProcessor(config, 1);
    VelodyneProcessor.loadTrajectoryAndPoses();
    VelodyneProcessor.loadAndProcessBinFiles();


    PointCloudProcessor AviaProcessor(config, 2);
    AviaProcessor.loadTrajectoryAndPoses();
    AviaProcessor.loadAndProcessBinFiles();

    PointCloudProcessor AevaProcessor(config, 3);
    AevaProcessor.loadTrajectoryAndPoses();
    AevaProcessor.loadAndProcessBinFiles();

  return 0;
}
