package frc.robot;

public final class Constants {
    public static class XboxControllerConstants {
        public static final int kxbox = 0;
    }
    public static class ShooterConstants {
        public static final int kUpPWMID = 1;
        public static final int kDownPWMID = 3;
        public static final Boolean kUpMotorInvert = false;
        public static final Boolean kDownMotorInvert = false;
    }

    public static class IntakeConstants {
        public static final int kintakeonePWMID = 2;
        public static final int kintaketwoPWMID = 0;
        public static final Boolean kintakeoneInvert = false;
        public static final Boolean kintaketwoInvert = false;
    }

    public static class RiseShooterConstants {
        public static final int kRiseShooterPWMID = 11;
        public static final Boolean kRiseShooterInvert = false;
        public static final double kriseVoltLimit = 4;
        public static final double kriseAngleMin = 0;
        public static final double kriseAngleMax = 45;
        public static final double kriseEncoderPulse = 2048;
        public static final double kriseInitAngleDegree = 0;
        public static final double kriseTriggerValue = 0.15;
    }

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
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
