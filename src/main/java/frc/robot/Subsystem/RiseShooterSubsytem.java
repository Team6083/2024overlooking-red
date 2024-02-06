// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RiseShooterConstants;

public class RiseShooterSubsytem extends SubsystemBase {
  /** Creates a new RiseShooterSubsytem. */
  private final VictorSPX riseMotor;
  private final Encoder riseEncoder;
  private double angleDegreeOffset;
  private final PIDController risePID;

  public RiseShooterSubsytem() {
    riseMotor = new VictorSPX(RiseShooterConstants.kRiseShooterPWMID);

    riseEncoder = new Encoder(0, 1);
    angleDegreeOffset = RiseShooterConstants.kriseInitAngleDegree;

    risePID = new PIDController(1, 0, 0);

    riseMotor.setInverted(RiseShooterConstants.kRiseShooterInvert);

    riseEncoder.setDistancePerPulse(360 / RiseShooterConstants.kriseEncoderPulse);
  }

  public void riseShooterControl(double RiseSpeed) {
    riseMotor.set(ControlMode.PercentOutput, RiseSpeed);
    risePID.setSetpoint(getAngleDegree());
  }

  public void pidControl() {
    double riseVolt = risePID.calculate(getAngleDegree());
    double modifiedRiseVolt = riseVolt;
    if (Math.abs(modifiedRiseVolt) > RiseShooterConstants.kriseVoltLimit) {
      modifiedRiseVolt = RiseShooterConstants.kriseVoltLimit * (riseVolt > 0 ? 1 : -1);
    }
    riseMotor.set(ControlMode.PercentOutput, riseVolt);

    SmartDashboard.putNumber("rise_volt", modifiedRiseVolt);
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
      setpoint = RiseShooterConstants.kriseAngleMin;
    } else if (isPhyLimitExceed(setpoint) == 1) {
      setpoint = RiseShooterConstants.kriseAngleMax;
    }
    risePID.setSetpoint(setpoint);

    // 
  }

  public double getAngleDegree() {
    SmartDashboard.putNumber("riseEncoderPos", riseEncoder.getDistance());
    return (riseEncoder.getDistance()) + angleDegreeOffset;
  }

  public void resetEncoder() {
    angleDegreeOffset = 0;
    riseEncoder.reset();
  }

  public void resetSetpoint() {
    risePID.setSetpoint(0);
  }

  public void stopMotor() {
    riseMotor.set(ControlMode.PercentOutput, 0.0);
  }

  private int isPhyLimitExceed(double angle) {
    return (angle < RiseShooterConstants.kriseAngleMin ? -1 : (angle > RiseShooterConstants.kriseAngleMax ? 1 : 0));
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("rise_PID", risePID);
  }
}
