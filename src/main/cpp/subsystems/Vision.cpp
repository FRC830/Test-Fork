#include <subsystems/Vision.h>

Vision::Vision(){
    camera.SetPipelineIndex(0);
    tag = camera.GetLatestResult();
    camera.SetPipelineIndex(1);
    tape = camera.GetLatestResult();
    camera.SetPipelineIndex(2);
    cone = camera.GetLatestResult();
    camera.SetPipelineIndex(3);
    cube = camera.GetLatestResult();
}

void updatePipeline(){
    camera.SetPipelineIndex(0);
    tag = camera.GetLatestResult();
    camera.SetPipelineIndex(1);
    tape = camera.GetLatestResult();
    camera.SetPipelineIndex(2);
    cone = camera.GetLatestResult();
    camera.SetPipelineIndex(3);
    cube = camera.GetLatestResult();
}

photonlib::PhotonPipelineResult getPipeline(std::string name) {

    switch(name) {
        case "tag":
            return tag;
        case "tape":
            return tape;
        case "cube":
            return cube;
        case "cone":
            return cone;
        default: 
            return
    }

}

