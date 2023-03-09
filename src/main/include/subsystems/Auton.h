#include "DriveSubsystem.h"

class Auton 
{


    public: 
        void runAuton(int mode);

    private:
        DriveSubsystem *m_drive; 
        void taxi ();
        int jimboGeeO = 12;

};