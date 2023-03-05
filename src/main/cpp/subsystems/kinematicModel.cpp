#include <Subsystems/kinematicModel.h>
#include <cstdlib> //for abs value function

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

    
