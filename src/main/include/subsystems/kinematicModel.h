class kinematicModel
{
    public:

    double robotAprilTagX;
    double robotAprilTagY;
    double robotAprilTagZ;
    double robotAprilTagYaw;
    
    double nodeAprilTagX;
    double nodeAprilTagY;
    double nodeAprilTagZ;

    double robotNodeX;
    double robotNodeY;
    double robotNodeZ;

    int aprilTagNum;

    void setNumbers();
    
    void calculateRobotNode();
    void getAprilTagPosOfSelectedNode(int nodeRow, int aprilTagNum);


    double robotNodeDistance;
    double robotAngleToNode;
    double robotAngleToFaceNode;
    double robotStrafeToNodeAngle;
    double robotStrafeToFaceNodeX;
    double robotStrafeToFaceNodeY;  
    
    double nodeDist;

    double armAngleFromHorizon;
    double armExtend;

    void calculateRobotAngles();
    void calculateArmPose();
    void calculateKinematics(int rowSelection);
    void setNumbers(double x, double y, double z, double yaw);


};
