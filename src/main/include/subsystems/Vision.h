#include <photonlib/PhotonCamera.h>

class Vision
{
private:
    /* data */
    photonlib::PhotonCamera camera{"photonvision"};
    

public:
    photonlib::PhotonPipelineResult tag;
    photonlib::PhotonPipelineResult tape;
    photonlib::PhotonPipelineResult cube;
    photonlib::PhotonPipelineResult cone;
    Vision();
    ~Vision();
    photonlib::PhotonPipelineResult getVisionOutput(std::string name);
    void updatePipeline();
    photonlib::PhotonPipelineResult getPipeline(std::string name);
    
};

Vision::Vision(){
}

