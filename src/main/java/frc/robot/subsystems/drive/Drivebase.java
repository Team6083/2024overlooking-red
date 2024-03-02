// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.NoteTrackingConstants;
// import frc.robot.subsystems.AprilTagTracking;
import frc.robot.subsystems.PowerDistributionSubsystem;
import frc.robot.subsystems.ApriltagTracking.TagTrackingLimelight;
import frc.robot.subsystems.NoteTracking.NoteTrackingPhotovision;

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

  private final AHRS gyro;

  private final PIDController facingNotePID;
  private final PIDController facingTagPID;
  private final PIDController followingTagPID;
  private final PIDController faceToSpecificAnglePID;

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

  private Boolean trackingCondition = false;

  private final PowerDistributionSubsystem powerDistributionSubsystem;
  private final NoteTrackingPhotovision note;
  private final TagTrackingLimelight aprilTagTracking;

  private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

  public Drivebase(PowerDistributionSubsystem powerDistributionSubsystem, NoteTrackingPhotovision note,
      TagTrackingLimelight aprilTagTracking) {
    this.powerDistributionSubsystem = powerDistributionSubsystem;
    this.note = note;
    this.aprilTagTracking = aprilTagTracking;
    frontLeftLocation = new Translation2d(0.3, 0.3);
    frontRightLocation = new Translation2d(0.3, -0.3);
    backLeftLocation = new Translation2d(-0.3, 0.3);
    backRightLocation = new Translation2d(-0.3, -0.3);

    frontLeft = new SwerveModule(10, 11, 5, DrivebaseConstants.kFrontLeftDriveMotorInverted);
    frontRight = new SwerveModule(12, 13, 4, DrivebaseConstants.kFrontRightDriveMotorInverted);
    backLeft = new SwerveModule(14, 15, 2, DrivebaseConstants.kBackLeftDriveMotorInverted);
    backRight = new SwerveModule(16, 17, 3, DrivebaseConstants.kBackRightDriveMotorInverted);

    SmartDashboard.putData("frontLeft", frontLeft);
    SmartDashboard.putData("frontRight", frontRight);
    SmartDashboard.putData("backLeft", backLeft);
    SmartDashboard.putData("backRight", backRight);

    gyro = new AHRS(Port.kMXP);

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

    // reset the gyro
    resetgyro();

    // set the swerve speed equal 0
    drive(0, 0, 0, false);

    facingNotePID = new PIDController(kP, kI, kD);
    followingTagPID = new PIDController(kfP, kfI, kfD);
    facingTagPID = new PIDController(kP, kI, kD);
    faceToSpecificAnglePID = new PIDController(kPP, kII, kDD);

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
            AutoConstants.drivebaseRadius, // Drive base radius in meters. Distance from robot center to furthest
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

  public void resetgyro() {
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
  public void drive(double ySpeed, double xSpeed, double rot, boolean fieldRelative) {
    swerveModuleStates = kinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, rot, gyro.getRotation2d())
            : new ChassisSpeeds(ySpeed, xSpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DrivebaseConstants.kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);
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

  public void faceTarget() {
    double offset = aprilTagTracking.getTx();
    double hasTarget = aprilTagTracking.getTv();
    double rot = 0;
    if (hasTarget == 1) {
      rot = facingTagPID.calculate(offset, 0);
    }
    drive(0, 0, -rot, false);
  }

  public double faceTargetMethod2() {
    double offset = aprilTagTracking.getTx();
    double hasTarget = aprilTagTracking.getTv();
    double rot = 0;
    if (hasTarget == 1) {
      rot = -facingTagPID.calculate(offset, 0);
    }
    SmartDashboard.putNumber("rot", rot);
    return rot;
  }

  public void follow() {
    double offset = aprilTagTracking.getTx();
    double hasTarget = aprilTagTracking.getTv();
    double rot = 0;
    if (hasTarget == 1) {
      rot = facingTagPID.calculate(offset, 0);
    }
    double[] bt = aprilTagTracking.getBT();
    double x_dis = bt[2];
    // double y_dis = bt[2];
    // double hasTarget = tag.getTv();
    double xSpeed = 0;
    // double ySpeed = 0;
    if (hasTarget == 1) {
      xSpeed = -followingTagPID.calculate(x_dis, 0.5);
      // ySpeed = follow_pid.calculate(y_dis, 1);
    }
    SmartDashboard.putNumber("x_dis_speed", xSpeed);
    // SmartDashboard.putNumber("y_dis_speed", ySpeed);
    drive(xSpeed, 0, -rot, false);
    SmartDashboard.putNumber("distance", aprilTagTracking.getMyDistance());
    // drive(0, 0, -rot, false);
  }

  public void fixDistanceBT() {
    double[] bt = aprilTagTracking.getBT();
    double x_dis = bt[0];
    double y_dis = bt[1];
    double hasTarget = aprilTagTracking.getTv();
    double xSpeed = 0;
    double ySpeed = 0;
    if (hasTarget == 1) {
      xSpeed = followingTagPID.calculate(x_dis, 0);
      ySpeed = followingTagPID.calculate(y_dis, 1);
    }
    SmartDashboard.putNumber("x_dis_speed", xSpeed);
    SmartDashboard.putNumber("y_dis_speed", ySpeed);
    drive(xSpeed, 0, 0, true);
  }

  public void fixDistanceCT() {
    double[] ct = aprilTagTracking.getCT();
    double x_dis = ct[0];
    double y_dis = ct[1];
    double hasTarget = aprilTagTracking.getTv();
    double xSpeed = 0;
    double ySpeed = 0;
    if (hasTarget == 1) {
      xSpeed = followingTagPID.calculate(x_dis, 0);
      ySpeed = followingTagPID.calculate(y_dis, 1);
    }
    SmartDashboard.putNumber("x_dis_speed", xSpeed);
    SmartDashboard.putNumber("y_dis_speed", ySpeed);
    drive(xSpeed, 0, 0, true);
  }

  public void switchTrackCondition() {
    trackingCondition = !trackingCondition;
  }

  public void addTrackTargetError() {
    noteTrackTargetError += 2;
  }

  public void minusTrackTargetError() {
    noteTrackTargetError -= 2;
  }

  public void resetTrackTargetError() {
    noteTrackTargetError = 0.0;
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

  public void putDashboard() {
    SmartDashboard.putNumber("frontLeft_speed", swerveModuleStates[0].speedMetersPerSecond);
    SmartDashboard.putNumber("frontRight_speed", swerveModuleStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("backLeft_speed", swerveModuleStates[2].speedMetersPerSecond);
    SmartDashboard.putNumber("backRight_speed", swerveModuleStates[3].speedMetersPerSecond);
    SmartDashboard.putNumber("gyro_heading", gyro.getRotation2d().getDegrees());
    SmartDashboard.putBoolean("trackingCondition", trackingCondition);
    aprilTagTracking.putDashboard();
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();
    putDashboard();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(frontLeft.getState(),
        frontRight.getState(),
        backLeft.getState(),
        backRight.getState());
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
            AutoConstants.drivebaseRadius, // Drive base radius in meters. Distance from robot center to furthest
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

  public Command followAutoCommand(String autoName) {
    return new PathPlannerAuto(autoName);
  }
}
