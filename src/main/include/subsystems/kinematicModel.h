class kinematicModel
{
    public:

    double robotAprilTagX=0;
    double robotAprilTagY=0;
    double robotAprilTagZ=0;
    double robotAprilTagYaw=0;
    double nodeAprilTagX=0;
    double nodeAprilTagY=0;
    double nodeAprilTagZ=0;

    double robotNodeX;
    double robotNodeY;
    double robotNodeZ;

    int aprilTagNum;
    
    void calculateRobotNode();
    void getAprilTagPosOfSelectedNode(int nodeRow, int aprilTagNum);


    double robotNodeDistance;
    double robotAngleToNode;
    double robotAngleToFaceNode;
    double robotStrafeToNodeAngle;
    double robotStrafeToFaceNodeX;
    double robotStrafeToFaceNodeY;    

    void calculateRobotAngles();
};
