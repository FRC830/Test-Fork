// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "Constants.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include <cmath>
#include <cstdio>
#include <string>

void Robot::RobotInit() noexcept
{
  frc::LiveWindow::SetEnabled(false);
  frc::LiveWindow::DisableAllTelemetry();

  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

    // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command; non-owning pointer is passed by value.
  auto driveRequirements = {dynamic_cast<frc2::Subsystem *>(&m_driveSubsystem)};

  // Drive, as commanded by operator joystick controls.
  m_driveCommand = std::make_unique<frc2::RunCommand>(
      [&]() -> void
      {
        if (m_lock)
        {
          (void)m_driveSubsystem.SetLockWheelsX();

          return;
        }

        const auto controls = GetDriveTeleopControls();

        m_driveSubsystem.Drive(
            std::get<0>(controls) * physical::kMaxDriveSpeed,
            std::get<1>(controls) * physical::kMaxDriveSpeed,
            std::get<2>(controls) * physical::kMaxTurnRate,
            std::get<3>(controls));
      },
      driveRequirements);

  // Point swerve modules, but do not actually drive.
  
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

  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  m_driveSubsystem.SetDefaultCommand(*m_driveCommand);
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


void Robot::ConfigureButtonBindings() noexcept
{
  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kA).WhenPressed(frc2::InstantCommand([&]() -> void
                                                                                                  { m_slow = true; },
                                                                                                  {}));
  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kB).WhenPressed(frc2::InstantCommand([&]() -> void
                                                                                                  { m_slow = false; },
                                                                                                  {}));

  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kX).WhenPressed(frc2::InstantCommand([&]() -> void
                                                                                                  { m_fieldOriented = false; },
                                                                                                  {}));
  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kY).WhenPressed(frc2::InstantCommand([&]() -> void
                                                                    
                                                                                                  { m_driveSubsystem.ZeroHeading();
                                                                                            m_fieldOriented = true; },
                                                                                                  {&m_driveSubsystem}));

 

}



std::tuple<double, double, double, bool> Robot::GetDriveTeleopControls() noexcept
{
  double x = -m_xbox.GetLeftY();
  double y = -m_xbox.GetLeftX();
  double z = -m_xbox.GetRightX();

  // between out = in^3.0 and out = in.
  auto shape = [](double raw, double mixer = 0.75) -> double
  {
    // Input deadband around 0.0 (+/- range).
    constexpr double range = 0.05;

    constexpr double slope = 1.0 / (1.0 - range);

    if (raw >= -range && raw <= +range)
    {
      raw = 0.0;
    }
    else if (raw < -range)
    {
      raw += range;
      raw *= slope;
    }
    else if (raw > +range)
    {
      raw -= range;
      raw *= slope;
    }

    return mixer * std::pow(raw, 3.0) + (1.0 - mixer) * raw;
  };

  x = shape(x);
  y = shape(y);
  z = shape(z, 0.0);

  if (m_slow)
  {
    x *= 0.50;
    y *= 0.50;
    z *= 0.40;
  }
  else
  { // XXX Still needed?
    x *= 2.0;
    y *= 2.0;
    z *= 1.6;
  }

  return std::make_tuple(x, y, z, m_fieldOriented);
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
