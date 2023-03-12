#include <Subsystems/kinematicModel.h>
#include <cstdlib> //for abs value function
#include <math.h>
// #include <cmath>

// calculates the position of a node based on the XYZ given by the april tags

void kinematicModel::setNumbers(){

    

}

void kinematicModel::calculateRobotNode() {
    robotNodeX = robotAprilTagX - nodeAprilTagX;
    robotNodeY = robotAprilTagY - nodeAprilTagY;
    robotNodeZ = robotAprilTagZ - nodeAprilTagZ;
};
    
void kinematicModel::getAprilTagPosOfSelectedNode(int nodeRow, int aprilTagNum) {

    double columnXtable[9]={22.125,0,-21.875,-43.875,-65.875,-87.875,-109.875,-131.875,-153.875};
    double rowYtable[3]={7.14,8.42,25.45};
    double rowZtable[3]={-18.16,7.84,27.65};
    double distances[9]={0,0,0,0,0,0,0,0,0};

    for(int i=0;i<=8;i++){
        double distance = abs(robotAprilTagX - (columnXtable[i] + 66*(3-aprilTagNum)));
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
    
    nodeAprilTagX=(columnXtable[shortestColumn] + 66*(3-aprilTagNum));
    nodeAprilTagY=rowYtable[nodeRow];
    nodeAprilTagZ=rowZtable[nodeRow];

};

// finds the angles required for the robot to strafe/rotate towards the node
void kinematicModel::calculateRobotAngles() {
    robotNodeDistance = hypot(robotNodeX, robotNodeY);
    robotAngleToNode = atan(robotNodeY / robotNodeX);

    // for rotate-then-strafe method
    // from here: robot_rotate(robotAprilTagYaw);
    //            robot_strafe(robotNodeX, 0);
    //            robot_arm_extend(robotNodeY);

    // for rotate-only method
    /*robotAngleToFaceNode = 90 + robotAprilTagYaw - robotAngleToNode;*/
    // from here: robot_rotate(robotAngleToFaceNode);
    //            robot_arm_extend(robotNodeDistance);

    // for strafe-only method
    robotStrafeToNodeAngle = robotAngleToNode + robotAprilTagYaw;
    robotStrafeToFaceNodeX = robotNodeDistance * cos(robotStrafeToNodeAngle);
    robotStrafeToFaceNodeY = robotNodeDistance * sin(robotStrafeToNodeAngle);

    nodeDist=robotStrafeToFaceNodeY;
    // from here: robot_strafe(robotStrafeToFaceNodeX, 0);
    //            robot_arm_extend(robotStrafeToFaceNodeY);
};
void kinematicModel::calculateArmPose(){
    armAngleFromHorizon=atan(-robotNodeZ/ nodeDist);
    armExtend=hypot(robotNodeZ, nodeDist);
};

void kinematicModel::calculateKinematics(int rowSelection){
    double robotAprilTagX=0;
    double robotAprilTagY=0;
    double robotAprilTagZ=0;
    double robotAprilTagYaw=0;
    double nodeAprilTagX=0;
    double nodeAprilTagY=0;
    double nodeAprilTagZ=0;

    //parse april tag
    //get robot pivot location

    getAprilTagPosOfSelectedNode(rowSelection, 2/*april tag output parsed to be between 1 and 3*/);
    calculateRobotNode();
    calculateRobotAngles();
    calculateArmPose();
}

//getters robot pose and position

//setters for initial conditions, aril tag ect


