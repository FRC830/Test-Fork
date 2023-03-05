#include "../include/subsystems/Subsystems.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>



void Subsystems::SubsystemsInit()
{

}

void Subsystems::SubsystemsPeriodic()
{
    ArmMotor.Set(ArmPIDController.Calculate(ArmMotorEncoder.GetPosition()));
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
void Subsystems::SetArmPIDF(double p, double i, double d, double f=0)
{
    ArmPIDController.SetP(p);
    ArmPIDController.SetI(i);
    ArmPIDController.SetD(d);
    //ArmPIDController.SetF(f);
}