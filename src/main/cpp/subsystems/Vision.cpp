#include <subsystems/Vision.h>
<<<<<<< HEAD
=======
#include <photonlib/PhotonCamera.h>
#include <photonlib/PhotonTrackedTarget.h>
#include <list>
>>>>>>> 8d2bd1b0fad9ec03207cbf70aca431a255dcc356

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

<<<<<<< HEAD
auto Vision::getListOfTags(){
=======
std::span<const photonlib::PhotonTrackedTarget> Vision::getListOfTags(){
>>>>>>> 8d2bd1b0fad9ec03207cbf70aca431a255dcc356

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