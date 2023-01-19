// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() noexcept
{
  frc::LiveWindow::SetEnabled(false);
  frc::LiveWindow::DisableAllTelemetry();

  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
// This performs the sense portion of sense/think/act, including sending test
// mode telemetry.  It also handles think and act, except in test mode.  In the
// latter case, TestPeriodic() handles manually driven act.
void Robot::RobotPeriodic() noexcept
{
  frc2::CommandScheduler::GetInstance().Run();
  
  frc::SmartDashboard::PutNumber("Robot Periodic", 1.0);
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */

void Robot::DisabledPeriodic() noexcept {}


/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() noexcept
{



}

void Robot::AutonomousPeriodic() noexcept {}

void Robot::AutonomousExit() noexcept {}

void Robot::TeleopInit() noexcept {

  frc::SmartDashboard::PutNumber("Death", 500);


  m_container.TeleopInit();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() noexcept {}

void Robot::TeleopExit() noexcept {}

void Robot::TestInit() noexcept {}

/**
 * This function is called periodically during test mode.
 * Note that test mode does not follow the command-based paradigm; it follows
 * Init/Periodic.
 */
void Robot::TestPeriodic() noexcept
{
}

void Robot::TestExit() noexcept {}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
