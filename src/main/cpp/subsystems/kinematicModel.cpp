#include <Subsystems/kinematicModel.h>

    void kinematicModel::calculateRobotNode() {
        robotNodeX = robotAprilTagX - nodeAprilTagX;
        robotNodeY = robotAprilTagY - nodeAprilTagY;
        robotNodeZ = robotAprilTagZ - nodeAprilTagZ;
    };
    
    void getAprilTagPosOfSelectedNode(int nodeRow) {
        
    };

    
