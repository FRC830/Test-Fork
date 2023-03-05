#include <Subsystems/kinematicModel.h>
#include <cmath>

// calculates the position of a node based on the XYZ given by the april tags
void kinematicModel::calculateRobotNode() {
    robotNodeX = robotAprilTagX - nodeAprilTagX;
    robotNodeY = robotAprilTagY - nodeAprilTagY;
    robotNodeZ = robotAprilTagZ - nodeAprilTagZ;
};
    
void getAprilTagPosOfSelectedNode(int nodeRow) {
    
};

// finds the angles required for the robot to strafe/rotate towards the node
void kinematicModel::calculateRobotAngles() {
    robotNodeDistance = cmath::hypot(robotNodeX, robotNodeY);
    robotAngleToNode = cmath::atan(robotNodeY / robotNodeX);

    // for rotate-then-strafe method
    // from here: robot_rotate(robotAprilTagYaw);
    //            robot_strafe(robotNodeX, 0);
    //            robot_arm_extend(robotNodeY);

    // for rotate-only method
    robotAngleToFaceNode = 90 + robotAprilTagYaw - robotAngleToNode;
    // from here: robot_rotate(robotAngleToFaceNode);
    //            robot_arm_extend(robotNodeDistance);

    // for strafe-only method
    robotStrafeToNodeAngle = robotAngleToNode - robotAprilTagYaw;
    robotStrafeToFaceNodeX = robotNodeDistance * cmath::cos(StrafeToNodeAngle);
    robotStrafeToFaceNodeY = robotNodeDistance * cmath::sin(StrafeToNodeAngle);
    // from here: robot_strafe(robotStrafeToFaceNodeX, 0);
    //            robot_arm_extend(robotStrafeToFaceNodeY);
};
