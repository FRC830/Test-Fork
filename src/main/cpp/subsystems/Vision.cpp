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
};

void Vision::updatePipeline(){
    camera.SetPipelineIndex(0);
    tag = camera.GetLatestResult();
    camera.SetPipelineIndex(1);
    tape = camera.GetLatestResult();
    camera.SetPipelineIndex(2);
    cone = camera.GetLatestResult();
    camera.SetPipelineIndex(3);
    cube = camera.GetLatestResult();
}

auto Vision::getListOfTags(){

    return tag.GetTargets();

}

photonlib::PhotonPipelineResult Vision::getPipeline(int num) {

    switch(num) {
        case 0:
            return tag;
        case 1:
            return tape;
        case 2:
            return cube;
        case 3:
            return cone;
        default: 
            return tag;
    }

}