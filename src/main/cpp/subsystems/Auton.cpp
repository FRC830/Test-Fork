#include "../include/subsystems/Auton.h"
#include <iostream>

#include "subsystems/DriveSubsystem.h"

void Auton::runAuton (int mode, auto m_drive, int counter)
{
    
    std::cout << "run auton" << std::endl;
    switch (mode)
    {
    case 0:
        taxi(m_drive, counter);
        break;
    
    case 1: 
        DockingRight(m_drive, counter);
    
    default:
        taxi(m_drive);
        break;
    }
}

void Auton::taxi(DriveSubsystem m_drive, int counter)
{
  if (counter < 275){

    m_drive.Drive(0.5_mps, 0_mps, 0_deg_per_s, false);
 
    }
}

void Auton::taxiWithLowScore(DriveSubsystem m_drive, int counter) {

    if(counter < 25) {

        m_drive.Drive(6_mps, 0_mps, 0_deg_per_s, false);

    } if (counter < 300) {

        m_drive.Drive(1.5_mps, 0_mps, 0_deg_per_s, false);

    }

}

void Auton::DockingRight(auto m_driveSubsystem, int counter) {
    if (counter < 275)
    {
        m_driveSubsystem.Drive(1.5_mps, 0_mps, 0_deg_per_s, false);
        
    } else if (counter < 325) {

        m_driveSubsystem.Drive(0_mps, 0_mps, 90_deg_per_s, false);

    } else if (counter < 375) {

        m_driveSubsystem.Drive(1_mps, 0_mps, -90_deg_per_s, false);

    } else if (counter < 425) {

        m_driveSubsystem.Drive(0_mps, 0_mps, -90_deg_per_s, false);

    } else if (counter < 475) {

        m_driveSubsystem.Drive(1.5_mps, 0_mps, 0_deg_per_s, false);

    }

}

