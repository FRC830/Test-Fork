#include "../include/subsystems/Subsystems.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>

Subsystems::Subsystems() : ArmPIDController (pidf::kArmP, pidf::kArmI, pidf::kArmD) 
{
}

void Subsystems::SubsystemsInit()
{
  TelescopeMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
  frc::SmartDashboard::PutNumber("ArmPCoefficient", pidf::kArmP);
  frc::SmartDashboard::PutNumber("ArmPAdder", 0);
  frc::SmartDashboard::PutNumber("ArmI", pidf::kArmI);
  frc::SmartDashboard::PutNumber("ArmD", pidf::kArmD);
  frc::SmartDashboard::PutNumber("Telescope Speed", 0.5);
    
}

void Subsystems::SubsystemsPeriodic()
{
    ArmMotor.Set(ArmPIDController.Calculate(ArmMotorEncoder.GetPosition()));
    SetArmPIDF
    (
        cos(ArmMotorEncoder.GetPosition() *(3.14 /180)) *
            frc::SmartDashboard::GetNumber("ArmPCoefficient", pidf::kArmP) +
            frc::SmartDashboard::GetNumber("ArmPAdder", 0), 
        frc::SmartDashboard::GetNumber("ArmI", pidf::kArmI), 
        frc::SmartDashboard::GetNumber("ArmD", pidf::kArmD)
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

void Subsystems::StopTelescope()
{

    TelescopeMotor.Set(0);
    frc::SmartDashboard::PutNumber("Telescope Speed", 0.5);

}

void Subsystems::moveTelescopethingy()
{
    auto speed = frc::SmartDashboard::GetNumber("Telescope Speed", 0.5);
    
    TelescopeMotor.Set(speed);
}