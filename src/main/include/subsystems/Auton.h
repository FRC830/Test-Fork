#include "DriveSubsystem.h"

class Auton 
{
    public: 
        void runAuton(int mode, DriveSubsystem* m_drive);

    private:
        void taxi (DriveSubsystem* m_drive);

};