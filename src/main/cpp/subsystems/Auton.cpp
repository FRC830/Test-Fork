#include "../include/subsystems/Auton.h"
#include <iostream>

void Auton::runAuton (int mode, DriveSubsystem* m_drive)
{
    
    std::cout << "run auton" << std::endl;
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
    std::cout << "taxi auton" << std::endl;
    m_drive->Drive(0.133_mps, 0_mps, 0_deg_per_s, false);
}