#include <frc/Solenoid.h>
#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include "../Constants.h"


class Subsystems : public frc2::SubsystemBase
{
    public:
        Subsystems();

        Subsystems(const Subsystems &) = delete;
        Subsystems &operator=(const Subsystems &) = delete;


        void SetGrabberWheels(bool direction);
        void ToggleGrabberPnumatics ();
        void RotateArm(bool direction);
        
        void SetArmPIDF (double p, double i, double d);
        void moveTelescopethingy();
        void StopTelescope();

        void SubsystemsInit ();
        void SubsystemsPeriodic ();
    private:
        //Constants
        double GrabberWheelSpeeds = 0.5;

        double ArmSpeed = 0.5;
        int MinArmAngle = 30;
        int MaxArmAngle = 250;
        int ArmAngleBufferSize = 20;

        double telescopeEncoderCurrentPosition=0;
        double telescopeEncoderMaxPosition=25;
        
        //Status Variables
        bool GrabberOnOff = false;

        //PID Control
        frc2::PIDController ArmPIDController;

        //Physical objects
        frc::Solenoid GrabberSolenoid{frc::PneumaticsModuleType::CTREPCM, 0};

        rev::CANSparkMax GrabberWheelsMotor =  rev::CANSparkMax(10, rev::CANSparkMaxLowLevel::MotorType::kBrushless);

        rev::CANSparkMax ArmMotor = rev::CANSparkMax(11, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
        rev::SparkMaxRelativeEncoder ArmMotorEncoder = ArmMotor.GetEncoder();

        rev::CANSparkMax TelescopeMotor = rev::CANSparkMax(12, rev::CANSparkMaxLowLevel::MotorType::kBrushless);

        

};