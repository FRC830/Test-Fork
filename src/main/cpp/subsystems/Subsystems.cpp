#include "../include/subsystems/Subsystems.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>

Subsystems::Subsystems() : ArmPIDController (pidf::kArmP, pidf::kArmI, pidf::kArmD), TelePIDController (pidf::kTeleP, pidf::kTeleI, pidf::kTeleD) 
{
}

void Subsystems::SubsystemsInit()
{
  frc::SmartDashboard::PutNumber("ArmPCoefficient", pidf::kArmP);
  frc::SmartDashboard::PutNumber("ArmPAdder", 0);
  frc::SmartDashboard::PutNumber("ArmI", pidf::kArmI);
  frc::SmartDashboard::PutNumber("ArmD", pidf::kArmD);
  ArmMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  frc::SmartDashboard::PutNumber("TeleP", pidf::kTeleP);
  frc::SmartDashboard::PutNumber("TeleI", pidf::kTeleI);
  frc::SmartDashboard::PutNumber("TeleD", pidf::kTeleD);
  teleMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    
}

void Subsystems::SubsystemsPeriodic()
{
    ArmMotor.Set(ArmPIDController.Calculate(ArmMotorEncoder.GetPosition()));
    SetArmPIDF
    (
        cos(ArmMotorEncoder.GetPosition() *(3.14 /180)) * TeleMotorEncoder.GetPosition() * 
            frc::SmartDashboard::GetNumber("ArmPCoefficient", pidf::kArmP) +
            frc::SmartDashboard::GetNumber("ArmPAdder", 0), 
        frc::SmartDashboard::GetNumber("ArmI", pidf::kArmI), 
        frc::SmartDashboard::GetNumber("ArmD", pidf::kArmD)
    );

    teleMotor.Set(TelePIDController.Calculate(TeleMotorEncoder.GetPosition()));
    SetTelePIDF
    (
        frc::SmartDashboard::GetNumber("TeleP", pidf::kTeleP), 
        frc::SmartDashboard::GetNumber("TeleI", pidf::kTeleI), 
        frc::SmartDashboard::GetNumber("TeleD", pidf::kTeleD)
    );
}

void Subsystems::SetGrabberWheels(bool direction)
{
    GrabberWheelsMotor.Set(GrabberWheelSpeeds * direction);
}
void Subsystems::ToggleGrabberPnumatics()
{
    if (GrabberOnOff)
    {
        GrabberSolenoid.Set(true);
    }
    else
    {
        GrabberSolenoid.Set(false);
    }

    GrabberOnOff = !GrabberOnOff;

}
void Subsystems::RotateArm(bool direction)
{
    auto setpoint = ArmPIDController.GetSetpoint();

    if (setpoint + ArmSpeed * direction < MaxArmAngle && setpoint + ArmSpeed * direction > MinArmAngle)//buffer could be applied here
    {
        ArmPIDController.SetSetpoint(setpoint + ArmSpeed);
    }
    else
    {
        std::cout << "Arm out of Bounds" << std::endl;
    }
}
void Subsystems::SetArmPIDF(double p, double i, double d)
{
    ArmPIDController.SetP(p);
    ArmPIDController.SetI(i);
    ArmPIDController.SetD(d);
    //ArmPIDController.SetF(f);
}

void Subsystems::SetTelePIDF(double p, double i, double d)
{
    TelePIDController.SetP(p);
    TelePIDController.SetI(i);
    TelePIDController.SetD(d);
    //ArmPIDController.SetF(f);
}

void Subsystems::moveTelescopethingy(bool direction)
{
    auto setpoint = TelePIDController.GetSetpoint();

    if (setpoint + TeleSpeed * direction < MaxArmAngle && setpoint +TeleSpeed * direction > MinArmAngle)//buffer could be applied here
    {
       TelePIDController.SetSetpoint(setpoint +TeleSpeed);
    }
    else
    {
        std::cout << "Arm out of Bounds" << std::endl;
    }
}