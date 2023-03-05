#include <Subsystems/kinematicModel.h>
#include <cstdlib> //for abs value function
#include <cmath>

// calculates the position of a node based on the XYZ given by the april tags
void kinematicModel::calculateRobotNode() {
    robotNodeX = robotAprilTagX - nodeAprilTagX;
    robotNodeY = robotAprilTagY - nodeAprilTagY;
    robotNodeZ = robotAprilTagZ - nodeAprilTagZ;
};
    
void kinematicModel::getAprilTagPosOfSelectedNode(int nodeRow, int aprilTagNum) {
    int nodeColumn;

    double table[9]={22.125,0,-21.875,-43.875,-65.875,-87.875,-109.875,-131.875,-153.875};
    double distances[9]={0,0,0,0,0,0,0,0,0};

    for(int i=0;i<=8;i++){
        double distance = abs(robotAprilTagX - table[i]);
        distances[i] = distance;
    }

    double shortestDistance = distances[0];
    int shortestColumn = 0;
    for(int i=1;i<=8;i++){
        if(distances[i]<shortestDistance) {
            shortestDistance = distances[i];
            shortestColumn = i;
        }
    }

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
