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
  private final double angleDegreeOffset;
  private final PIDController risePID;

  public RiseShooterSubsystem() {
    riseMotor = new CANSparkMax(RiseShooterConstants.kRiseShooterChannel, MotorType.kBrushless);

    riseEncoder = new Encoder(0, 1);
    angleDegreeOffset = RiseShooterConstants.kRiseInitAngleDegree;

    risePID = new PIDController(1, 0, 0);

    riseMotor.setInverted(RiseShooterConstants.kRiseShooterInverted);

    riseMotor.setSmartCurrentLimit(40);

    riseEncoder.setDistancePerPulse(360.0 / RiseShooterConstants.kRiseEncoderPulse);
  }

  public void manualControl(double riseSpeed) {
    riseMotor.set(riseSpeed);
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
    riseMotor.set(riseVolt);

    SmartDashboard.putNumber("rise_volt", modifiedRiseVolt);
  }

  public double getAngleDegree() {
    double degree =  ((riseEncoder.getDistance()) + angleDegreeOffset)%360.0;
    SmartDashboard.putNumber("riseShooterDegree", degree);
    return degree;
  }

  public void resetEncoder() {
    riseEncoder.reset();
  }

  public void resetSetpoint() {
    risePID.setSetpoint(0);
  }

  public void stopMotor() {
    riseMotor.set(0.0);
  }

  private int isPhyLimitExceed(double angle) {
    return (angle < RiseShooterConstants.kRiseAngleMin ? -1 : (angle > RiseShooterConstants.kRiseAngleMax ? 1 : 0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("rise_PID", risePID);
  }
}
