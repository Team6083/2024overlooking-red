package frc.robot;

public final class Constants {
    public static class XboxControllerConstants {
        public static final int kMain = 0;
    }

    public static class ShooterConstants {
        public static final int kShooterUpChannel = 3;
        public static final int kShooterDownChannel = 2;
        public static final Boolean kUpMotorInverted = false;
        public static final Boolean kDownMotorInverted = false;
        public static final double kUpMotorPower = 0.8;
        public static final double kDownMotorPower = 0.8;
    }

    public static class TransportConstants{
        public static final int kTrantsportChannel = 1;
    }

    public static class IntakeConstants {
        public static final int kIntakeUpChannel = 4;
        public static final int kIntakeDownChannel = 0;
        public static final double kIntakePrecentage = -0.4;
        public static final Boolean kIntakeUpInverted = false;
        public static final Boolean kIntakeDownInverted = true;
    }

    public static class RiseShooterConstants {
        public static final int kRiseShooterChannel = 11;
        public static final Boolean kRiseShooterInverted = false;
        public static final double kRiseVoltLimit = 4;
        public static final double kRiseAngleMin = 0;
        public static final double kRiseAngleMax = 45;
        public static final double kRiseEncoderPulse = 2048;
        public static final double kRiseInitAngleDegree = 0;
        public static final double kRiseTriggerValue = 0.15;
    }

    public static class HookConstants{
        public static final int kHookLineChannel = 12;
    }

    public static class DrivebaseConstants {
        public static final int kFrontLeftDriveMotorChannel = 10;
        public static final int kFrontRightDriveMotorChannel = 12;
        public static final int kBackLeftDriveMotorChannel = 14;
        public static final int kBackRightDriveMotorChannel = 16;

        // turning motor channel
        public static final int kFrontLeftTurningMotorChannel = 11;
        public static final int kFrontRightTurningMotorChannel = 13;
        public static final int kBackLeftTurningMotorChannel = 15;
        public static final int kBackRightTurningMotorChannel = 17;

        // turnning encoder channel
        public static final int kFrontLeftTurningEncoderChannel = 5;
        public static final int kFrontRightTurningEncoderChannel = 4;
        public static final int kBackLeftTurningEncoderChannel = 2;
        public static final int kBackRightTurningEncoderChannel = 3;

        // can coder magnet offset value
        public static final double kFrontLeftCanCoderMagOffset = 0.066650;
        public static final double kFrontRightCanCoderMagOffset = -0.442871;
        public static final double kBackLeftCanCoderMagOffset = 0.351562;
        public static final double kBackRightCanCoderMagOffset = -0.333740;

        public static final double kMaxSpeed = 3;
        public static final double kMinSpeed = 0.25;
        public static final double kMinJoyStickValue = 0.3;
        public static final double kMaxAngularSpeed = 2.5 * Math.PI; // 1/2 rotation per second

        public static final double xLimiterRateLimit = 3.0;
        public static final double yLimiterRateLimit = 3.0;
        public static final double rotLimiterRateLimit = 3.0;

        public static final boolean kFrontLeftDriveMotorInverted = true;
        public static final boolean kFrontRightDriveMotorInverted = false;
        public static final boolean kBackLeftDriveMotorInverted = true;
        public static final boolean kBackRightDriveMotorInverted = false;

        public static final boolean kGyroInverted = false; // wheather gyro is under the robot
        public static final double kGyroOffSet = -90.0;
    }

    public static final class ModuleConstants {
        public static final double kWheelRadius = 0.046;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kLimitModuleDriveVoltage = 7.0;
        public static final double kMaxModuleDriveVoltage = 12.0;
        public static final double kClosedLoopRampRate = 0.25;// 1 second 1 unit
        public static final double kDesireSpeedtoMotorVoltage = kMaxModuleDriveVoltage / DrivebaseConstants.kMaxSpeed;
        public static final double kMaxModuleTuringVoltage = 7.0;
        public static final double kMaxSpeedTurningDegree = 180.0;
        public static final double kPRotController = kMaxModuleTuringVoltage / kMaxSpeedTurningDegree;
        public static final double kDRotController = 0.0004;
    }


    }

