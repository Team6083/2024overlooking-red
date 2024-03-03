// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RiseShooterConstants;
import frc.robot.subsystems.ApriltagTracking.TagTrackingLimelight;

public class RiseShooterSubsystem extends SubsystemBase {

  /** Creates a new RiseShooterSubsytem. */
  private final CANSparkMax riseMotor;
  private final DutyCycleEncoder riseEncoder;
  private final double angleDegreeOffset;
  private final PIDController risePID;
  private final PowerDistributionSubsystem powerDistribution;
  private final TagTrackingLimelight tagTrackingLimelight;

  public RiseShooterSubsystem(PowerDistributionSubsystem powerDistribution, TagTrackingLimelight aprilTagTracking) {
    riseMotor = new CANSparkMax(RiseShooterConstants.kRiseShooterChannel, MotorType.kBrushless);

    riseEncoder = new DutyCycleEncoder(RiseShooterConstants.kEncoderChannel);
    angleDegreeOffset = RiseShooterConstants.kRiseAngleOffset;

    risePID = new PIDController(RiseShooterConstants.kP, RiseShooterConstants.kI, RiseShooterConstants.kD);

    riseMotor.setInverted(RiseShooterConstants.kRiseShooterInverted);

    this.powerDistribution = powerDistribution;
    this.tagTrackingLimelight = aprilTagTracking;
    setSetpoint(60.0);
    risePID.enableContinuousInput(-180.0, 180.0);
  }

  public void manualControl(double RiseSpeed) {
    setMotor(RiseSpeed);
    risePID.setSetpoint(getAngleDegree());
  }

  public double getSetpoint() {
    return risePID.getSetpoint();
  }

  public void setSetpoint(double setpoint) {
    final double currentSetpoint = getSetpoint();
    if (isPhyLimitExceed(currentSetpoint) != 0) {
      return;
    }
    if (isPhyLimitExceed(setpoint) == -1) {
      setpoint = RiseShooterConstants.kRiseAngleMin;
    } else if (isPhyLimitExceed(setpoint) == 1) {
      setpoint = RiseShooterConstants.kRiseAngleMax;
    }
    risePID.setSetpoint(setpoint);
  }

  public void pidControl() {
    double riseVolt = risePID.calculate(getAngleDegree());
    double modifiedRiseVolt = riseVolt;
    if (Math.abs(modifiedRiseVolt) > RiseShooterConstants.kRiseVoltLimit) {
      modifiedRiseVolt = RiseShooterConstants.kRiseVoltLimit * (riseVolt > 0 ? 1 : -1);
    }
    setMotor(riseVolt);
    SmartDashboard.putNumber("rise_volt", modifiedRiseVolt);
  }

  public double getAprilTagDegree() {
    if (tagTrackingLimelight.getTv() == 1) {
      return Math.toDegrees(Math.atan(RiseShooterConstants.kSpeakerHeight / tagTrackingLimelight.getBT()[2]));
    } else {
      return getAngleDegree();
    }

  }

  public double getAngleDegree() {
    double degree = (RiseShooterConstants.kEncoderInverted ? -1.0 : 1.0)
        * (riseEncoder.getAbsolutePosition() - angleDegreeOffset) * 360.0;
    SmartDashboard.putNumber("riseShooterDegree", degree);
    return degree;
  }

  public void addError(double error) {

  }

  public void resetEncoder() {
    riseEncoder.reset();
  }

  public void resetSetpoint() {
    risePID.setSetpoint(0);
  }

  public void stopMotor() {
    riseMotor.setVoltage(0.0);
  }

  public void setMotor(double voltage) {
    if (powerDistribution.isRiseShooterOverCurrent()) {
      stopMotor();
      return;
    }
    riseMotor.setVoltage(voltage);
  }

  private int isPhyLimitExceed(double angle) {
    return (angle < RiseShooterConstants.kRiseAngleMin ? -1 : (angle > RiseShooterConstants.kRiseAngleMax ? 1 : 0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("rise_PID", risePID);
    SmartDashboard.putNumber("motor", riseMotor.getOutputCurrent());
  }
}
