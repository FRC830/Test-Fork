#include "DriveSubsystem.h"

class Auton 
{


    public: 
        void runAuton (int mode, auto m_drive, int counter);

    private:
        void taxi(DriveSubsystem m_drive, int counter);
        void DockingRight(auto m_drive, int counter);
        

};