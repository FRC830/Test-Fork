
#include <photonlib/PhotonCamera.h>


using namespace photonlib;
class Vision
{
private:
    /* data */
    
public:


    Vision();

    photonlib::PhotonCamera camera{"photonvision"};
    photonlib::PhotonPipelineResult tag, tape, cube, cone;


    void updatePipeline();
    photonlib::PhotonPipelineResult getVisionOutput(std::string name);
    photonlib::PhotonPipelineResult getPipeline(int num);
    auto getListOfTags();
};
