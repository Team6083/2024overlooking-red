package frc.robot;

public final class Constants {
    public static class DriveControllerConstants {
        public static final int kMainController = 0;
        public static final int kControlPanel = 2;
    }

    public static class ShooterConstants {
        public static final int kUpMotorChannel = 3;
        public static final int kDownMotorChannel = 5;
        public static final int kUpEncoderChannelA = 2;
        public static final int kUpEncoderChannelB = 3;
        public static final int kDownEncoderChannelA = 5;
        public static final int kDownEncoderChannelB = 6;
        public static final Boolean kUpMotorInverted = false;
        public static final Boolean kDownMotorInverted = false;
        public static final Boolean kUpEncoderInverted = true;
        public static final Boolean kDownEncoderInverted = true;
        public static final double kUpMotorManualVoltage = 10.0;
        public static final double kDownMotorManualVoltage = 10.0;
        public static final double kShooterRate = 55.0;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kUpMotorS = 0.756;
        public static final double kUpMotorV = 0.149;
        public static final double kUpMotorA = 0.0;
        public static final double kDownMotorS = 0.856;
        public static final double kDownMotorV = 0.149;
        public static final double kDownMotorA = 0.0;
    }

    public static class TransportConstants {
        public static final int kTrantsportChannel = 4;
        public static final boolean kTransportInverted = true;
        public static final double kTransSpeed = 0.8;
        public static final double kReTransSpeed = -0.5;
        public static final double kDistRange = 8.0;
    }

    public static class IntakeConstants {
        public static final int kIntakeChannel = 2;
        public static final Boolean kIntakeInverted = true;
        public static final double kIntakePrecentage = 0.65;
        public static final double kThrowPrecentage = -0.4;
    }

    public static class RiseShooterConstants {
        public static final int kRiseShooterChannel = 21;
        public static final Boolean kRiseShooterInverted = true;
        public static final int kEncoderChannelA = 0;
        public static final int kEncoderChannelB = 1;
        public static final double kRiseVoltLimit = 5;
        public static final double kRiseAngleMin = -5;
        public static final double kRiseAngleMax = 65;
        public static final double kRiseEncoderPulse = 2048;
        public static final double kRiseInitAngleDegree = 60;
        public static final double kRiseTriggerValue = 0.15;
        public static final double kSpeakerHeight = 5.0;
        public static final double kP = 0.5;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static class HookConstants {
        public static final int kHookLineChannel = 22;
        public static final int kHookLeftMotorCnannel = 1;
        public static final int kHookRightMotorCnannel = 2;
        public static final int kHookLeftEncoderChannelA=0;
        public static final int kHookLeftEncoderChannelB=1;
        public static final int kHookRightEncoderChannelA=2;
        public static final int kHookRightEncoderChannelB=3;
        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kLinePositionMax = 60.0;
        public static final double kLinePositionMin = 0.0;
        public static final double kLeftPositionMax=60.0;
        public static final double kLeftPositionMin=0.0;
        public static final double kRightPositionMax=60.0;
        public static final double kRightPositionMin=0.0;
        public static final double kHookPositionConversionfactor = 1.0;
        public static final double kLinePower = 0;
        public static final double kHookMotorLeftVoltage = 0.0;
        public static final double kHookMotorRightVoltage = 0.0;
        public static final double kmanualControlLeftHookMotorPower=0.25;
        public static final double kmanualControlRightHookMotorPower=0.25;
        public static final boolean kHookMotorLeftInverted = false;
        public static final boolean kHookMotorRightInverted = false;
        public static final double kInitSetpoint = 0.0;
        public static final double kLeftMotorUpModify=0.0;//未定值需更改
        public static final double kLeftMotorDownModify=0.0;
        public static final double kRightMotorUpModify=0.0;
        public static final double kRightMotorDownModify=0.0;
    }

    public static class DrivebaseConstants {
        public static final int kFrontLeftDriveMotorChannel = 11;
        public static final int kFrontRightDriveMotorChannel = 15;
        public static final int kBackLeftDriveMotorChannel = 13;
        public static final int kBackRightDriveMotorChannel = 17;

        // turning motor channel
        public static final int kFrontLeftTurningMotorChannel = 12;
        public static final int kFrontRightTurningMotorChannel = 16;
        public static final int kBackLeftTurningMotorChannel = 14;
        public static final int kBackRightTurningMotorChannel = 18;

        // turnning encoder channel
        public static final int kFrontLeftTurningEncoderChannel = 31;
        public static final int kFrontRightTurningEncoderChannel = 32;
        public static final int kBackLeftTurningEncoderChannel = 33;
        public static final int kBackRightTurningEncoderChannel = 34;

        // can coder magnet offset value
        public static final double kFrontLeftCanCoderMagOffset = -0.060303;
        public static final double kFrontRightCanCoderMagOffset = -0.442871;
        public static final double kBackLeftCanCoderMagOffset = 0.351562;
        public static final double kBackRightCanCoderMagOffset = -0.333740;

        public static final double kMaxSpeed = 3;
        public static final double kMinSpeed = 0.25;
        public static final double kMinJoyStickValue = 0.3;
        public static final double kMaxAngularSpeed = 2.5 * Math.PI; // 1/2 rotation per second

        public static final double kXLimiterRateLimit = 3.0;
        public static final double kYLimiterRateLimit = 3.0;
        public static final double kRotLimiterRateLimit = 3.0;

        public static final boolean kFrontLeftDriveMotorInverted = true;
        public static final boolean kFrontRightDriveMotorInverted = false;
        public static final boolean kBackLeftDriveMotorInverted = true;
        public static final boolean kBackRightDriveMotorInverted = false;

        public static final double kPNoteTrackingValue = 1.0;
        public static final boolean kGyroInverted = false; // wheather gyro is under the robot
        public static final double kGyroOffSet = 0;
    }

    public static final class ModuleConstants {
        public static final double kWheelRadius = 0.046;
        public static final double kWheelDiameterMeters = 0.15;
        public static final double kMaxModuleDriveVoltage = 12.0;
        public static final double kClosedLoopRampRate = 0.25;// 1 second 1 unit
        public static final double kDesireSpeedtoMotorVoltage = kMaxModuleDriveVoltage / DrivebaseConstants.kMaxSpeed;
        public static final double kMaxModuleTuringVoltage = 7.0;
        public static final double kMaxSpeedTurningDegree = 180.0;
        public static final double kPRotController = kMaxModuleTuringVoltage / kMaxSpeedTurningDegree;
        public static final double kDRotController = 0.0004;
    }

    public static final class PdConstants {
        public static final int kIntakeMotorCurrrentchannel = 0;
        public static final int kShooterDownMotorCurrentchannel = 1;
        public static final int kShooterUpMotorCurrentchannel = 2;
        public static final int kLineCurrentchannel = 3;
        public static final int kHookMotor1Currentchannel = 4;
        public static final int kHookMotor2Currentchannel = 5;
        public static final int kTransportCurrentchannel = 0;
        public static final int kRiseShooterCurrentchannel = 1;

        public static final double kIntakeMotorMaxCurrent = 0;
        public static final double kShooterDownMotorMaxCuurent = 0;
        public static final double kShooterUpMotorMaxCurrent = 0;
        public static final double kLineMotorMaxCurrent = 0;
        public static final double kHookMotor1MaxCurrent = 0;
        public static final double kHookMotor2MaxCurrent = 0;
        public static final double kTransportMaxCurrent = 0;
        public static final double kRiseShooterMaxCurrent = 40.0;
    }

    public static final class AutoConstants {
        // chooser path name
        public static final String leftTrans = "LeftTrans";
        public static final String left = "Left";
        public static final String middle = "Middle";
        public static final String rightTrans = "RightTrans";

        // constants
        public static final double kPTranslation = 2.0;
        public static final double kITranslation = 0.0;
        public static final double kDTranslation = 0.0;
        public static final double kPRotation = 2.0;
        public static final double kIRotation = 0.0;
        public static final double kDRotation = 0.0;
        public static final double drivebaseRadius = 0.34;
    }

    public static final class NoteTrackingConstants {
        public static final String kCameraName = "Microsoft_LifeCam_HD-3000";
        public static final int noteTrakingPipeline = 1;
        public static final double cameraHeight = 0.36;
        public static final double cameraWeight = 0.0;
        public static final double pitchDegree = -20.0;
        public static final double yawDegree = 0;
        public static final double minNoteDistance = 0.2;
    }

    public static final class AprilTagConstants {
        public static final int A_pipeline = 0;
        public static final double klimelightLensHeightInches = 0;
        public static final double klimelightMountAngleDegrees = 0;
    }
}
