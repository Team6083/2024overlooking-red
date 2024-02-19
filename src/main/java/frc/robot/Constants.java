package frc.robot;

public final class Constants {
    public static class XboxControllerConstants {
        public static final int kMain = 0;
    }

    public static class ShooterConstants {
        public static final int kShooterUpChannel = 3;
        public static final int kShooterDownChannel = 5;
        public static final Boolean kUpMotorInverted = false;
        public static final Boolean kDownMotorInverted = false;
        public static final Boolean kUpEncoderInverted = true;
        public static final Boolean kDownEncoderInverted = true;
        public static final double kUpMotorPower = 0.8;
        public static final double kDownMotorPower = 0.8;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.756;
        public static final double kV = 0.149;
        public static final double kA = 0.0;
    }

    public static class TransportConstants {
        public static final int kTrantsportChannel = 4;
        public static final double kTransSpeed = 0.3;
        public static final double kReTransSpeed = -0.3;
        public static final double kDistRange = 8.0;
    }

    public static class IntakeConstants {
        public static final int kIntakeUpChannel = 2;
        public static final int kIntakeDownChannel = 1;
        public static final double kIntakePrecentage = 0.4;
        public static final double kThrowPrecentage = -0.4;
        public static final Boolean kIntakeUpInverted = true;
        public static final Boolean kIntakeDownInverted = false;
    }

    public static class RiseShooterConstants {
        public static final int kRiseShooterChannel = 21;
        public static final Boolean kRiseShooterInverted = false;
        public static final double kRiseVoltLimit = 4;
        public static final double kRiseAngleMin = 0;
        public static final double kRiseAngleMax = 45;
        public static final double kRiseEncoderPulse = 2048;
        public static final double kRiseInitAngleDegree = 0;
        public static final double kRiseTriggerValue = 0.15;
    }

    public static class HookConstants {
        public static final int kHookLineChannel = 22;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kHookPositionMax = 60.0;
        public static final double kHookPositionMin = 0.0;
        public static final double kHookPower = 0;
        public static final boolean kHookMotorInverted = false;
        public static final double kInitSetpoint = 0.0;

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

    public static final class VisionTrackingConstants {
        public static final int A_pipeline = 0;
        public static final double klimelightLensHeightInches = 0;
        public static final double klimelightMountAngleDegrees = 0;
    }

    public static final class AutoConstants {
        // chooser path name

        // constants
        public static final double kPTranslation = 2.0;
        public static final double kITranslation = 0.0;
        public static final double kDTranslation = 0.0;
        public static final double kPRotation = 2.0;
        public static final double kIRotation = 0.0;
        public static final double kDRotation = 0.0;
        public static final double maxModuleSpeed = 3.0;
        public static final double drivebaseRadius = 0.34;
      }

}
