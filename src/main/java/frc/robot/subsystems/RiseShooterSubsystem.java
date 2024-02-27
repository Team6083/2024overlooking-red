// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RiseShooterConstants;


public class RiseShooterSubsystem extends SubsystemBase {

  /** Creates a new RiseShooterSubsytem. */
  private final CANSparkMax riseMotor;
  private final Encoder riseEncoder;
  private double angleDegreeOffset;
  private final PIDController risePID;

  public RiseShooterSubsystem() {
    riseMotor = new CANSparkMax(RiseShooterConstants.kRiseShooterChannel, MotorType.kBrushless);

    riseEncoder = new Encoder(0, 1);
    angleDegreeOffset = RiseShooterConstants.kRiseInitAngleDegree;

    risePID = new PIDController(0.5, 0, 0);

    risePID.setSetpoint(60.0);

    riseMotor.setInverted(RiseShooterConstants.kRiseShooterInverted);

    riseEncoder.setDistancePerPulse(360.0 / RiseShooterConstants.kRiseEncoderPulse);
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
      risePID.setSetpoint((isPhyLimitExceed(currentSetpoint)) == 1 ? RiseShooterConstants.kRiseAngleMax
          : RiseShooterConstants.kRiseAngleMin);
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
    double distance = Math.pow(Math.pow(AprilTagTracking.getBT()[0], 2)+Math.pow(AprilTagTracking.getBT()[2], 2), 1/2);
    return Math.toDegrees(Math.atan(RiseShooterConstants.kSpeakerHeight / distance));
  }

  public double getAngleDegree() {
    double degree = (riseEncoder.getDistance()+angleDegreeOffset)%360.0;
    SmartDashboard.putNumber("riseShooterDegree", degree);
    return degree;
  }

  public void resetEncoder() {
    angleDegreeOffset = 0;
    riseEncoder.reset();
  }

  public void resetSetpoint() {
    risePID.setSetpoint(0);
  }

  public void stopMotor() {
    riseMotor.setVoltage(0.0);
  }

  public void setMotor(double power){
  if(powerDistributionSubsystem.isRiseShooterOverCurrent()){
    stopMotor();
    return;
  }
  riseMotor.setVoltage(power);
  }

  private int isPhyLimitExceed(double angle) {
    return (angle < RiseShooterConstants.kRiseAngleMin ? -1 : (angle > RiseShooterConstants.kRiseAngleMax ? 1 : 0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("rise_PID", risePID);
    SmartDashboard.putNumber("motor",riseMotor.getOutputCurrent());
  }
}
