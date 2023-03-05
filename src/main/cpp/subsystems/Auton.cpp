#include "../include/subsystems/Auton.h"

void Auton::runAuton (int mode, DriveSubsystem* m_drive)
{
    switch (mode)
    {
    case 0:
        taxi(m_drive);
        break;
    
    default:
        break;
    }
}

void Auton::taxi(DriveSubsystem* m_drive)
{
    m_drive->Drive(0.133_mps, 0_mps, 0_deg_per_s, false);
}