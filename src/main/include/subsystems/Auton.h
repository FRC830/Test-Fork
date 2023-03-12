#include "DriveSubsystem.h"

class Auton 
{


    public: 
        void runAuton (int mode, DriveSubsystem& m_drive, int counter);

    private:
        void taxi(DriveSubsystem& m_drive, int counter);
        void taxiWithLowScore(DriveSubsystem& m_drive, int counter);
        void DockingRight(DriveSubsystem& m_drive, int counter);
        void DockingLeft(DriveSubsystem& m_drive, int counter);

        

};