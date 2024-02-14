// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  private final VictorSPX shootUpMotor;
  private final VictorSPX shootDownMotor;
  private final Encoder upEncoder;
  private final Encoder downEncoder;
  private final PIDController ratePidController;
  private final SimpleMotorFeedforward rateFeedForwardControl;

  public ShooterSubsystem() {

    shootUpMotor = new VictorSPX(ShooterConstants.kShooterUpChannel);
    shootDownMotor = new VictorSPX(ShooterConstants.kShooterDownChannel);

    upEncoder = new Encoder(2, 3);
    downEncoder = new Encoder(4, 5);

    upEncoder.setReverseDirection(ShooterConstants.kUpEncoderInverted);
    downEncoder.setReverseDirection(ShooterConstants.kDownEncoderInverted);

    ratePidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

    shootUpMotor.setInverted(ShooterConstants.kUpMotorInverted);
    shootDownMotor.setInverted(ShooterConstants.kDownMotorInverted);

    rateFeedForwardControl = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);

    resetEncoder();
    SmartDashboard.putNumber("shooter_rate", 0);
    SmartDashboard.putNumber("shooter_Voltage", 0);
  }

  public void setManual() {
    double voltage = 10;
    double upPower = voltage / getUpMotorBusVoltage();
    double downPower = voltage / getDownMotorBusVoltage();
    shootUpMotor.set(VictorSPXControlMode.PercentOutput, upPower);
    shootDownMotor.set(VictorSPXControlMode.PercentOutput, downPower);
  }

  public void resetEncoder() {
    upEncoder.reset();
    downEncoder.reset();
  }

  public void setSetpoint(double distance) {
    double rate = SmartDashboard.getNumber("shooter_rate", 0.0);
    ratePidController.setSetpoint(rate);
  }

  public void setRateToPower() {
    double rate = SmartDashboard.getNumber("shooter_rate", 0.0);
    final double rateToUpMotorPower = (rateFeedForwardControl.calculate(rate)
        + ratePidController.calculate(getUpEncoderRate())) / getUpMotorBusVoltage();
    final double rateToDownMotorPower = (rateFeedForwardControl.calculate(rate)
        + ratePidController.calculate(getDownEncoderRate())) / getDownMotorBusVoltage();
    shootUpMotor.set(VictorSPXControlMode.PercentOutput, rateToUpMotorPower);
    shootDownMotor.set(VictorSPXControlMode.PercentOutput, rateToDownMotorPower);
    SmartDashboard.putNumber("rateToUpMotorPower", rateToUpMotorPower);
    SmartDashboard.putNumber("rateToDownMotorPower", rateToDownMotorPower);
  }

  public void stopMotor() {
    shootUpMotor.set(VictorSPXControlMode.PercentOutput, 0);
    shootDownMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public double getUpEncoderRate() {
    return upEncoder.getRate() / 2048.0;
  }

  public double getDownEncoderRate() {
    return downEncoder.getRate() / 2048.0;
  }

  public double getUpMotorBusVoltage() {
    return shootUpMotor.getBusVoltage();
  }

  public double getDownMotorBusVoltage() {
    return shootDownMotor.getBusVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("upRate", getUpEncoderRate());
    SmartDashboard.putNumber("downRate", getDownEncoderRate());
    SmartDashboard.putNumber("upPower", shootUpMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("downPower", shootDownMotor.getMotorOutputPercent());
  }
}
