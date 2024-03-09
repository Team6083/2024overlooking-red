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
import frc.robot.subsystems.apriltagTracking.TagTrackingLimelight;

public class RotateShooterSubsystem extends SubsystemBase {

  /** Creates a new RiseShooterSubsytem. */
  private final CANSparkMax rotateMotor ;
  private final DutyCycleEncoder rotateEncoder;
  private final double angleDegreeOffset;
  private final PIDController rotatePID;
  private double rotateDegreeError = 0.0;
  private final PowerDistributionSubsystem powerDistributionSubsystem;
  private final TagTrackingLimelight tagTrackingLimelight;
  // private final SparkMaxRelativeEncoder riseEncoderSPX;

  public RotateShooterSubsystem(PowerDistributionSubsystem powerDistributionSubsystem, TagTrackingLimelight aprilTagTracking) {
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

  public void fineTurnUpShooter(){
    rotateEncoder.setPositionOffset(angleDegreeOffset);
  }

  public void fineTurnDownShooter(){
     rotateEncoder.setPositionOffset(-angleDegreeOffset);
  }

  public double getAprilTagDegree(double currentSetpoint) {
    if (tagTrackingLimelight.getTv() == 1) {
      double horizontalDistance = Math.abs(tagTrackingLimelight.getBT()[2]) - 0.21*Math.cos(Math.toRadians(getAngleDegree()));
      double degree = Math.toDegrees(Math.atan((1.6+Math.sin(Math.toRadians(getAngleDegree())) / horizontalDistance)));
      return degree;
    } else {
      return currentSetpoint;
    }

  }

  public double getAngleDegree() {
    double degree = (RotateShooterConstants.kEncoderInverted ? -1.0 : 1.0)
        * ((rotateEncoder.getAbsolutePosition() * 360.0) - 251.0);
    // double degreeSPX = ((RiseShooterSubsystem) riseEncoderSPX).getAngleDegree();
    // double degreeFin = degree-degreeSPX;
    // if(degree-degreeSPX>1){
    //   degreeFin=degree;
    // }else{
    //   degreeFin = degree-degreeSPX;
    // }
    SmartDashboard.putNumber("rotateShooterDegree", degree);
    return degree;
  }

public double getShooterToTagHoriDis(){
  // shooter to cam z dis, remember to move this to constant later on
  double offset = 0.11;
  double z_dis = offset + tagTrackingLimelight.getBT()[2];
  double x_dis = tagTrackingLimelight.getBT()[0];
  double horDis = Math.sqrt(Math.pow(x_dis, 2)+Math.pow(z_dis, 2));
  return horDis;
}
  
  public void detectAngle(){
    
  }

  public Command addErrorCommand(double error) {
    return Commands.run(() -> addError(error), this);
  }
  
  public void addError(double error) {
    rotateDegreeError = error*RotateShooterConstants.kRiseDegreeErrorPoint;
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
