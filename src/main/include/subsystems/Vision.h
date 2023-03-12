#include <photonlib>

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
    List<photonlib::PhotonTrackedTarget> getListOfTags();
};
