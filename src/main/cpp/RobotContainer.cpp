// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Constants.h"
#include "RobotContainer.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>

#include <cmath>
#include <cstdio>
#include <string>

RobotContainer::RobotContainer() noexcept
{
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

void RobotContainer::AutonomousInit() noexcept
{
  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  m_driveSubsystem.SetDefaultCommand(frc2::RunCommand([&]() -> void {},
                                                      {&m_driveSubsystem}));
}

void RobotContainer::TeleopInit() noexcept
{
  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  m_driveSubsystem.SetDefaultCommand(*m_driveCommand);

}

void RobotContainer::ConfigureButtonBindings() noexcept
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



std::tuple<double, double, double, bool> RobotContainer::GetDriveTeleopControls() noexcept
{
  // The robot's frame of reference is the standard unit circle, from
  // trigonometry.  However, the front of the robot is facing along the positve
  // X axis.  This means the poitive Y axis extends outward from the left (or
  // port) side of the robot.  Poitive rotation is counter-clockwise.  On the
  // other hand, as the controller is held, the Y axis is aligned with forward.
  // And, specifically, it is the negative Y axis which extends forward.  So,
  // the robot's X is the controllers inverted Y.  On the controller, the X
  // axis lines up with the robot's Y axis.  And, the controller's positive X
  // extends to the right.  So, the robot's Y is the controller's inverted X.
  // Finally, the other controller joystick is used for commanding rotation and
  // things work out so that this is also an inverted X axis.
  double x = -m_xbox.GetLeftY();
  double y = -m_xbox.GetLeftX();
  double z = -m_xbox.GetRightX();

  // PlayStation controllers seem to do this strange thing with the rotation:
  // double z = -m_xbox.GetLeftTriggerAxis();
  // Note: there is now a PS4Controller class.

  // Add some deadzone, so the robot doesn't drive when the joysticks are
  // released and return to "zero".  These implement a continuous deadband, one
  // in which the full range of outputs may be generated, once joysticks move
  // outside the deadband.

  // Also, cube the result, to provide more opertor control.  Just cubing the
  // raw value does a pretty good job with the deadband, but doing both is easy
  // and guarantees no movement in the deadband.  Cubing makes it easier to
  // command smaller/slower movements, while still being able to command full
  // power.  The 'mixer` parameter is used to shape the `raw` input, some mix
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