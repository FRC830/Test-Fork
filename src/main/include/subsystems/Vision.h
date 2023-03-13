#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonTrackedTarget.h>
#include <photonlib/PhotonPipelineResult.h>
#include <list>

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
    std::span<const photonlib::PhotonTrackedTarget> getListOfTags();
};
