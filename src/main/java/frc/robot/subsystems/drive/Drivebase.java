// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.NoteTrackingConstants;
import frc.robot.Constants.TagTrackingConstants;
import frc.robot.subsystems.visionProcessing.NoteTracking;
import frc.robot.subsystems.visionProcessing.TagTracking;

public class Drivebase extends SubsystemBase {
  /** Creates a new Drivetain. */
  private final Translation2d frontLeftLocation;
  private final Translation2d frontRightLocation;
  private final Translation2d backLeftLocation;
  private final Translation2d backRightLocation;

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  // private final AHRS gyro2;
  private final Pigeon2 gyro;
  // private final ADXRS450_Gyro gyro;

  private final Field2d field2d;

  private double magnification;

  private final PIDController facingNotePID;
  private final PIDController facingTagPID;
  private final PIDController followingTagPID;
  private final PIDController faceToSpecificAnglePID;
  private final PIDController drivePID;

  // face method value maybe correct
  private final double kP = 0.08;
  private final double kI = 0;
  private final double kD = 0;

  // fix distance value not determined yet
  private final double kfP = 0.8;
  private final double kfI = 0;
  private final double kfD = 0.006;

  // fix position
  public static final double kPP = 0.03;
  public static final double kII = 0;
  public static final double kDD = 0;

  private double noteTrackTargetError = 0.0;

  private boolean noteTrackingCondition = false;
  private boolean tagTrackingCondition = false;

  private final NoteTracking note;
  private final TagTracking aprilTagTracking;

  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

  public Drivebase(NoteTracking noteTracking,
      TagTracking aprilTagTracking) {
    this.note = noteTracking;
    this.aprilTagTracking = aprilTagTracking;
    frontLeftLocation = new Translation2d(0.3, 0.3);
    frontRightLocation = new Translation2d(0.3, -0.3);
    backLeftLocation = new Translation2d(-0.3, 0.3);
    backRightLocation = new Translation2d(-0.3, -0.3);

    frontLeft = new SwerveModule(DrivebaseConstants.kFrontLeftDriveMotorChannel,
        DrivebaseConstants.kFrontLeftTurningMotorChannel, DrivebaseConstants.kFrontLeftTurningEncoderChannel,
        DrivebaseConstants.kFrontLeftDriveMotorInverted, DrivebaseConstants.kFrontLeftCanCoderMagOffset, "frontLeft");
    frontRight = new SwerveModule(DrivebaseConstants.kFrontRightDriveMotorChannel,
        DrivebaseConstants.kFrontRightTurningMotorChannel, DrivebaseConstants.kFrontRightTurningEncoderChannel,
        DrivebaseConstants.kFrontRightDriveMotorInverted, DrivebaseConstants.kFrontRightCanCoderMagOffset,
        "frontRight");
    backLeft = new SwerveModule(DrivebaseConstants.kBackLeftDriveMotorChannel,
        DrivebaseConstants.kBackLeftTurningMotorChannel, DrivebaseConstants.kBackLeftTurningEncoderChannel,
        DrivebaseConstants.kBackLeftDriveMotorInverted, DrivebaseConstants.kBackLeftCanCoderMagOffset, "backLeft");
    backRight = new SwerveModule(DrivebaseConstants.kBackRightDriveMotorChannel,
        DrivebaseConstants.kBackRightTurningMotorChannel, DrivebaseConstants.kBackRightTurningEncoderChannel,
        DrivebaseConstants.kBackRightDriveMotorInverted, DrivebaseConstants.kBackRightCanCoderMagOffset, "backRight");

    SmartDashboard.putData("frontLeft", frontLeft);
    SmartDashboard.putData("frontRight", frontRight);
    SmartDashboard.putData("backLeft", backLeft);
    SmartDashboard.putData("backRight", backRight);

    // gyro = new AHRS(Port.kMXP);
    // gyro = new ADXRS450_Gyro();
    gyro = new Pigeon2(30);

    kinematics = new SwerveDriveKinematics(
        frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

    // create the odometry
    odometry = new SwerveDriveOdometry(
        kinematics,
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
    field2d = new Field2d();

    // initialize magnification value
    magnification = DrivebaseConstants.kDefaultMagnification;

    // reset the gyro
    resetGyro();

    // set the swerve speed equal 0
    drive(0, 0, 0, false);

    facingNotePID = new PIDController(kP, kI, kD);
    followingTagPID = new PIDController(kfP, kfI, kfD);
    facingTagPID = new PIDController(kP, kI, kD);
    faceToSpecificAnglePID = new PIDController(kPP, kII, kDD);
    drivePID = new PIDController(0, 0, 0);

    AutoBuilder.configureHolonomic(
        this::getPose2d, // Robot pose suppier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(AutoConstants.kPTranslation, AutoConstants.kITranslation, AutoConstants.kDTranslation), // Translation
                                                                                                                     // PID
                                                                                                                     // constants
            new PIDConstants(AutoConstants.kPRotation, AutoConstants.kIRotation, AutoConstants.kDRotation), // Rotation
                                                                                                            // PID
                                                                                                            // constants
            DrivebaseConstants.kMaxSpeed, // Max module speed, in m/s
            AutoConstants.kDrivebaseRadius, // Drive base radius in meters. Distance from robot center to furthest
                                            // module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        pose);
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(DrivebaseConstants.kGyroOffSet
        + ((DrivebaseConstants.kGyroInverted) ? (360.0 - gyro.getRotation2d().getDegrees())
            : gyro.getRotation2d().getDegrees()));
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param ySpeed        Speed of the robot in the y direction (forward).
   * @param xSpeed        Speed of the robot in the x direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * 
   *                      using the wpi function to set the speed of the swerve
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    if (noteTrackingCondition) {
      rot = facingNoteRot(rot);
    }
    if (tagTrackingCondition) {
      rot = facingTag(rot);
    }
    swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivebaseConstants.kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
  }

  public Command accelerateCmd() {
    Command cmd = Commands.runOnce(() -> magnification = DrivebaseConstants.kHighMagnification, this);
    return cmd;
  }

  public Command defaultSpeedCmd() {
    Command cmd = Commands.runOnce(() -> magnification = DrivebaseConstants.kDefaultMagnification, this);
    return cmd;
  }

  public void setMagnification(double magnification) {
    this.magnification = magnification;
  }

  public double getMagnification() {
    return magnification;
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
  }

  public Pose2d getPose2d() {
    return odometry.getPoseMeters();
  }

  public void resetRobotPose2d() {
    frontLeft.resetAllEncoder();
    frontRight.resetAllEncoder();
    backLeft.resetAllEncoder();
    backRight.resetAllEncoder();
  }

  public boolean hasTargets() {
    return note.hasTargets();
  }

  public double facingNoteRot(double currentRot) {
    var target = note.getNotes();
    if (target.size() > 0) {
      var pose = target.get(0);
      double rot = -facingNotePID.calculate(pose.getX(), 0);
      return rot;
    } else {
      return currentRot;
    }
  }

  public double[] followingNoteSpeed() {
    var target = note.getNotes();
    double[] speed = new double[3];
    speed[0] = 0;
    speed[1] = 0;
    speed[2] = 0;
    if (target.size() > 0) {
      var pose = target.get(0);
      double XSpeed = facingNotePID.calculate(pose.getY(), NoteTrackingConstants.minNoteDistance);
      double YSpeed = 0;
      double rot = -facingNotePID.calculate(pose.getX(), 0);
      speed[0] = XSpeed;
      speed[1] = YSpeed;
      speed[2] = rot;
    }
    return speed;
  }

  public double facingTag(double currentRot) {
    double offset = aprilTagTracking.getTx();
    double hasTarget = aprilTagTracking.getTv();
    double targetID = aprilTagTracking.getTID();
    if (hasTarget == 1 && targetID != 3.0 && targetID != 8.0) {
      double rot = -facingTagPID.calculate(offset, 0);
      return rot;
    }
    return currentRot;
  }

  /**
   * Return a double array. [0] xSpeed, [1] ySpeed, [2] rot
   * 
   * @return follow (double array)
   */
  public double[] followingTag() {
    double offset = aprilTagTracking.getTx();
    double hasTarget = aprilTagTracking.getTv();
    double[] speed = new double[3];
    double xSpeed = 0;
    double ySpeed = 0;
    double rot = 0;
    double x_dis = aprilTagTracking.getBT()[2];
    if (hasTarget == 1) {
      rot = facingTagPID.calculate(offset, 0);
      xSpeed = -followingTagPID.calculate(x_dis, 0.5);
    }
    speed[0] = xSpeed;
    speed[1] = ySpeed;
    speed[2] = rot;
    return speed;
  }

  public void switchNoteTrackCondition() {
    noteTrackingCondition = !noteTrackingCondition;
  }

  public void switchTagTrackCondition() {
    tagTrackingCondition = !tagTrackingCondition;
  }

  public Command noteTrackCondition() {
    return Commands.runOnce(() -> switchNoteTrackCondition());
  }

  public Command tagTrackConditionCmd() {
    return Commands.runOnce(() -> switchTagTrackCondition());
  }

  public void driveToSpecificPose2d(Pose2d desiredPose) {
    Pose2d botpose = getPose2d();
    Transform2d offset = desiredPose.minus(botpose);
    double xoffset = offset.getX();
    double yoffset = offset.getY();
    double roffset = offset.getRotation().getDegrees();
    SmartDashboard.putNumber("xoffset", xoffset);
    SmartDashboard.putNumber("yoffset", yoffset);
    SmartDashboard.putNumber("aoffset", roffset);
    double xSpeed = drivePID.calculate(xoffset);
    double ySpeed = drivePID.calculate(yoffset);
    double rot = drivePID.calculate(roffset);
    drive(xSpeed, ySpeed, rot, true);
  }

  /**
   * Drvie to stage by a button. 
   * @param choice three modes: middle, left and right. All Lower case.
   */ 
  public void driveToStage(String choice) {
    Transform2d offset = new Transform2d();
    switch (choice) {
      case "middle":
        Rotation2d rot1 = new Rotation2d(Math.toRadians(180));
        offset = new Transform2d(0.42, 0, rot1);
        break;
      case "left":
        Rotation2d rot2 = new Rotation2d(Math.toRadians(180));
        offset = new Transform2d(0.42, 0.7, rot2);
        break;
      case "right":
        Rotation2d rot3 = new Rotation2d(Math.toRadians(180));
        offset = new Transform2d(0.42, -0.7, rot3);
        break;
    }
    Pose2d tagPose = aprilTagTracking.getDesiredTagPose2d(aprilTagTracking.getLastID());
    Pose2d goalPose = tagPose.plus(offset);
    driveToSpecificPose2d(goalPose);
  }

  public Command driveToStageMiddleCmd(){
    return Commands.runOnce(()-> driveToStage("middle"));
  }

  public Command driveToStageRightCmd(){
    return Commands.runOnce(()->driveToStage("right"));
  }
  
  public Command driveToStageLeftCmd(){
    return Commands.runOnce(()->driveToStage("left"));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    // photonTracking.putDashboard();
    field2d.setRobotPose(getPose2d());
    // putDashboard();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("driveBase");
    builder.addDoubleProperty("frontLeft_speed", () -> swerveModuleStates[0].speedMetersPerSecond, null);
    builder.addDoubleProperty("frontRight_speed", () -> swerveModuleStates[1].speedMetersPerSecond, null);
    builder.addDoubleProperty("backLeft_speed", () -> swerveModuleStates[2].speedMetersPerSecond, null);
    builder.addDoubleProperty("backRight_speed", () -> swerveModuleStates[3].speedMetersPerSecond, null);
    builder.addDoubleProperty("gyro_heading", () -> gyro.getRotation2d().getDegrees(), null);
    builder.addBooleanProperty("trackingCondition", () -> noteTrackingCondition, null);
    builder.addDoubleProperty("GyroResetCmd", null, null);
    builder.addDoubleProperty("PoseResetCmd", null, null);
    aprilTagTracking.putDashboard();
  }

  // public void putDashboard() {
  // SmartDashboard.putNumber("frontLeft_speed",
  // swerveModuleStates[0].speedMetersPerSecond);
  // SmartDashboard.putNumber("frontRight_speed",
  // swerveModuleStates[1].speedMetersPerSecond);
  // SmartDashboard.putNumber("backLeft_speed",
  // swerveModuleStates[2].speedMetersPerSecond);
  // SmartDashboard.putNumber("backRight_speed",
  // swerveModuleStates[3].speedMetersPerSecond);
  // SmartDashboard.putNumber("gyro_heading", gyro.getRotation2d().getDegrees());
  // SmartDashboard.putBoolean("trackingCondition", trackingCondition);
  // aprilTagTracking.putDashboard();
  // SmartDashboard.putData(GyroResetCmd());
  // SmartDashboard.putData(PoseResetCmd());
  // }

  public Command GyroResetCmd() {
    Command cmd = Commands.runOnce(this::resetGyro, this);
    cmd.setName("GyroResetCmd");
    return cmd;
  }

  public Command PoseResetCmd() {
    Command cmd = Commands.runOnce(this::resetPose2dAndEncoder, this);
    cmd.setName("PoseResetCmd");
    return cmd;
  }

  public Boolean checkPose2d(Pose2d targetPose2d) {
    Pose2d currentPose2d = getPose2d();
    if (Math.abs(currentPose2d.getX() - targetPose2d.getX()) < 30
        && Math.abs(currentPose2d.getY() - targetPose2d.getY()) < 15
        && Math.abs(targetPose2d.getRotation().getDegrees() - currentPose2d.getRotation().getDegrees()) < 5) {
      return true;
    }
    return false;
  }

  public void resetPose2dAndEncoder() {
    frontLeft.resetAllEncoder();
    frontRight.resetAllEncoder();
    backLeft.resetAllEncoder();
    backRight.resetAllEncoder();
    resetPose(new Pose2d(0, 0, new Rotation2d(0)));
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
  }

  public double calShooterAngleByPose2d() {
    double x = getPose2d().getX();
    double y = getPose2d().getY();
    double distance = Math.sqrt(x * x + y * y);
    double backAngleDegree = Math.toDegrees(Math.atan((FieldConstants.speakerBackTall - 32.464) / distance));
    double frontAngleDegree = Math.toDegrees(Math.atan((FieldConstants.speakerFrontTall - 32.464) / distance));
    return (backAngleDegree + frontAngleDegree) / 2;
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false);
  }

  // auto drive
  public Command followPathCommand(String pathName) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

    return new FollowPathHolonomic(
        path,
        this::getPose2d, // Robot pose supplier
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(AutoConstants.kPTranslation, AutoConstants.kITranslation, AutoConstants.kDTranslation), // Translation
                                                                                                                     // PID
                                                                                                                     // constants
            new PIDConstants(AutoConstants.kPRotation, AutoConstants.kIRotation, AutoConstants.kDRotation), // Rotation
                                                                                                            // PID
                                                                                                            // constants
            DrivebaseConstants.kMaxSpeed, // Max module speed, in m/s
            AutoConstants.kDrivebaseRadius, // Drive base radius in meters. Distance from robot center to furthest
                                            // module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirementsme
    );
  }

  public Command pathFindingThenFollowPath(String pathName, double maxVelocity, double maxAcceleration,
      double maxAngularVelocity, double maxAngularAcceleration, double rotationDelayDistance) {
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    PathConstraints constraints = new PathConstraints(
        maxVelocity, maxAcceleration,
        Units.degreesToRadians(maxAngularVelocity), Units.degreesToRadians(maxAngularAcceleration));
    return AutoBuilder.pathfindThenFollowPath(path, constraints, rotationDelayDistance);
  }

  public Command pathFindingToPose(double x, double y, double degrees, double maxVelocity, double maxAcceleration,
      double maxAngularVelocity, double maxAngularAcceleration, double goalEndVelocity,
      double rotationDelayDistance) {

    Pose2d targetPose = new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
    PathConstraints constraints = new PathConstraints(
        maxVelocity, maxAcceleration,
        Units.degreesToRadians(maxAngularVelocity), Units.degreesToRadians(maxAngularAcceleration));
    return AutoBuilder.pathfindToPose(targetPose, constraints, goalEndVelocity, rotationDelayDistance);
  }
}
