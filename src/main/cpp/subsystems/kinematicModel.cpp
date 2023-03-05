#include <Subsystems/kinematicModel.h>
#include <cmath>

void kinematicModel::calculateRobotNode() {
    robotNodeX = robotAprilTagX - nodeAprilTagX;
    robotNodeY = robotAprilTagY - nodeAprilTagY;
    robotNodeZ = robotAprilTagZ - nodeAprilTagZ;
};
    
void getAprilTagPosOfSelectedNode(int nodeRow) {
    
};

void kinematicModel::calculateRobotAngles() {
    robotNodeDistance = cmath::hypot(robotNodeX, robotNodeY);
    robotAngleToNode = cmath::atan(robotNodeY / robotNodeX);
    robotAngleToFaceNode = 90 + robotAprilTagYaw - robotAngleToNode;
    robotStrafeToNodeAngle = robotAngleToNode - robotAprilTagYaw;
    robotStrafeToFaceNodeX = robotNodeDistance * cmath::cos(StrafeToNodeAngle);
    robotStrafeToFaceNodeY = robotNodeDistance * cmath::sin(StrafeToNodeAngle);

};

    
