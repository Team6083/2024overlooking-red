// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.SparkMaxAbsoluteEncoder;
// import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RotateShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.visionProcessing.TagTracking;

public class RotateShooterSubsystem extends SubsystemBase {

  /** Creates a new RiseShooterSubsytem. */
  private final CANSparkMax rotateMotor;
  private final DutyCycleEncoder rotateEncoder;
  private final double angleDegreeOffset;
  private final PIDController rotatePID;
  private double rotateDegreeError = 0.0;
  private final PowerDistributionSubsystem powerDistributionSubsystem;
  private final TagTracking tagTrackingLimelight;
  // private final SparkMaxRelativeEncoder riseEncoderSPX;

  public RotateShooterSubsystem(PowerDistributionSubsystem powerDistributionSubsystem,
      TagTracking aprilTagTracking) {
    rotateMotor = new CANSparkMax(RotateShooterConstants.kRotateShooterChannel, MotorType.kBrushless);

    rotateEncoder = new DutyCycleEncoder(RotateShooterConstants.kEncoderChannel);
    angleDegreeOffset = RotateShooterConstants.kRiseAngleOffset;

    rotatePID = new PIDController(RotateShooterConstants.kP, RotateShooterConstants.kI, RotateShooterConstants.kD);

    rotateMotor.setInverted(RotateShooterConstants.kRotateShooterInverted);
    this.powerDistributionSubsystem = powerDistributionSubsystem;
    this.tagTrackingLimelight = aprilTagTracking;
    setSetpoint(angleDegreeOffset);
    rotatePID.enableContinuousInput(-180.0, 180.0);
  }

  public void manualControl(double RotateSpeed) {
    setMotor(RotateSpeed);
    rotatePID.setSetpoint(getAngleDegree());
  }

  public double getSetpoint() {
    return rotatePID.getSetpoint();
  }

  public void setSetpoint(double setpoint) {
    final double currentSetpoint = getSetpoint();
    if (hasExceedPhysicalLimit(currentSetpoint) != 0) {
      return;
    }
    if (hasExceedPhysicalLimit(setpoint) == -1) {
      setpoint = RotateShooterConstants.kRiseAngleMin;
    } else if (hasExceedPhysicalLimit(setpoint) == 1) {
      setpoint = RotateShooterConstants.kRiseAngleMax;
    }
    rotatePID.setSetpoint(setpoint);
  }

  public void pidControl() {
    double rotateVoltage = rotatePID.calculate(getAngleDegree());
    double modifiedRotateVoltage = rotateVoltage;
    if (Math.abs(modifiedRotateVoltage) > RotateShooterConstants.kRiseVoltLimit) {
      modifiedRotateVoltage = RotateShooterConstants.kRiseVoltLimit * (rotateVoltage > 0 ? 1 : -1);
    }
    setMotor(rotateVoltage);
    SmartDashboard.putNumber("rise_volt", modifiedRotateVoltage);
  }

  public void fineTurnUpShooter() {
    rotateEncoder.setPositionOffset(angleDegreeOffset);
  }

  public void fineTurnDownShooter() {
    rotateEncoder.setPositionOffset(-angleDegreeOffset);
  }

  public double getAprilTagDegree(double currentSetpoint) {
    if (tagTrackingLimelight.getTv() == 1) {
      double horizontalDistance = getShooterToTagHoriDis();
      double degree = Math.toDegrees(Math.atan(
          (1.6 + Math.sin(Math.toRadians(getAngleDegree()))
              / horizontalDistance)));
      return degree;
    } else {
      return currentSetpoint;
    }
  }

  public double getAprilTagDegree2(double currentSetpoint) {
    if (tagTrackingLimelight.getTv() == 1) {
      double horizontalDistance = getShooterToTagHoriDis();
      // constants shooter hegiht
      double degree = Math.toDegrees(Math.atan(
          (1.6 + Math.sin(Math.toRadians(getAngleDegree()))
              / horizontalDistance)));
      return degree;
    } else {
      return currentSetpoint;
    }
  }

  public double getAngleDegree() {
    double degree = (RotateShooterConstants.kEncoderInverted ? -1.0 : 1.0)
        * ((rotateEncoder.getAbsolutePosition() * 360.0) - 251.0);
    SmartDashboard.putNumber("rotateShooterDegree", degree);
    return degree;
  }

  public double getShooterToTagHoriDis() {
    // shooter to cam z dis, remember to move this to constant later on
    double offset = VisionConstants.CamToShooterOffset;
    double z_dis = offset + tagTrackingLimelight.getCT()[2];
    double x_dis = tagTrackingLimelight.getCT()[0];
    double horDis = Math.sqrt(Math.pow(x_dis, 2) + Math.pow(z_dis, 2));
    return horDis;
  }

  public double getGoalHeight() {
    double tagHeight = tagTrackingLimelight.getTagPose3d().getY();
    double goalTagOffset = VisionConstants.SpeakerOpeningToTagHeight; // 1 should be a constant of speaker opening to tag in metres.
    double offset = tagTrackingLimelight.getCT()[1] - VisionConstants.CamShooterHeight;
    double height = tagHeight + goalTagOffset - offset;
    return height;
  }

  public double getDistance() {
    double dis = Math.sqrt(Math.pow(getGoalHeight(), 2) + Math.pow(getShooterToTagHoriDis(), 2));
    return dis;
  }

  /**
   * Get the angle between the shooter anthor and the speaker opening. Can be used
   * for rotate shooter's setpoint
   * 
   * @return angle (degree)
   */
  public double getDesiredAngle() {
    double angle = Math.toDegrees(Math.acos(getShooterToTagHoriDis() / getDistance()));
    return angle;
  }

  public Command addErrorCommand(double error) {
    return Commands.run(() -> addError(error), this);
  }

  public void addError(double error) {
    rotateDegreeError = error * RotateShooterConstants.kRiseDegreeErrorPoint;
  }

  public void resetEncoder() {
    rotateEncoder.reset();
  }

  public void resetSetpoint() {
    rotatePID.setSetpoint(0);
  }

  public void stopMotor() {
    rotateMotor.setVoltage(0.0);
  }

  public void setMotor(double voltage) {
    if (powerDistributionSubsystem.isRotateShooterOverCurrent()) {
      stopMotor();
      return;
    }
    rotateMotor.setVoltage(voltage);
  }

  private int hasExceedPhysicalLimit(double angle) {
    return (angle < RotateShooterConstants.kRiseAngleMin ? -1 : (angle > RotateShooterConstants.kRiseAngleMax ? 1 : 0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("rotate_PID", rotatePID);
    SmartDashboard.putNumber("rotate_motor", rotateMotor.getOutputCurrent());
  }
}
