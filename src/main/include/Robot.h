// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/CommandPtr.h>

#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc2/command/RunCommand.h>

#include "subsystems/DriveSubsystem.h"

#include <memory>
#include <tuple>

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() noexcept override;
  void RobotPeriodic() noexcept override;
  // void DisabledInit() noexcept override;
  void DisabledPeriodic() noexcept override;
  //void DisabledExit() noexcept override;
  void AutonomousInit() noexcept override;
  void AutonomousPeriodic() noexcept override;
  void AutonomousExit() noexcept override;
  void TeleopInit() noexcept override;
  void TeleopPeriodic() noexcept override;
  void TeleopExit() noexcept override;
  void TestInit() noexcept override;
  void TestPeriodic() noexcept override;
  void TestExit() noexcept override;

private:
  std::optional<frc2::CommandPtr> m_autonomousCommand;

  std::tuple<double, double, double, bool> GetDriveTeleopControls() noexcept;

  void ConfigureButtonBindings() noexcept;

  bool m_fieldOriented{false};
  bool m_lock{false};
  bool m_slow{false};

  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_driveSubsystem;

  std::unique_ptr<frc2::RunCommand> m_driveCommand;
  std::unique_ptr<frc2::RunCommand> m_pointCommand;
  
  frc::XboxController m_xbox{0};
  frc::GenericHID m_buttonBoard{1};
};
