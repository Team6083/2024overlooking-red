// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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
  private final VictorSP shootUpMotor;
  private final VictorSP shootDownMotor;
  private final Encoder upEncoder;
  private final Encoder downEncoder;
  private final PIDController ratePidController;
  private final SimpleMotorFeedforward rateFeedForwardControl;

  public ShooterSubsystem() {

    shootUpMotor = new VictorSP(ShooterConstants.kShooterUpChannel);
    shootDownMotor = new VictorSP(ShooterConstants.kShooterDownChannel);

    upEncoder = new Encoder(0, 1);
    downEncoder = new Encoder(2, 3);

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
    shootUpMotor.setVoltage(voltage);
    shootDownMotor.setVoltage(voltage);
  }

  public void resetEncoder() {
    upEncoder.reset();
    downEncoder.reset();
  }

  public void setSetpoint(double distance) {
    ratePidController.setSetpoint(distance);
  }

  public void setFeedForwardRate() {
    double rate = SmartDashboard.getNumber("shooter_rate", 0.0);
    rate = rateFeedForwardControl.calculate(rate);
    shootUpMotor.setVoltage(rate);
    shootDownMotor.setVoltage(rate);
  }

  public void setPIDRate() {
    final double rateToUpMotorPower = ratePidController.calculate(getUpEncoderRate());
    final double rateToDownMotorPower = ratePidController.calculate(getDownEncoderRate());
    shootUpMotor.set(rateToUpMotorPower);
    shootDownMotor.set(rateToDownMotorPower);
    SmartDashboard.putNumber("rateToUpMotorPower", rateToUpMotorPower);
    SmartDashboard.putNumber("rateToDownMotorPower", rateToDownMotorPower);
  }

  public void stopMotor() {
    shootUpMotor.set(0);
    shootDownMotor.set(0);
  }

  public double getUpEncoderRate(){
    return upEncoder.getRate()/2048.0;
  }

  public double getDownEncoderRate(){
    return downEncoder.getRate()/2048.0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("upRate", getUpEncoderRate());
    SmartDashboard.putNumber("downRate", getDownEncoderRate());
    SmartDashboard.putNumber("upPower", shootUpMotor.get());
    SmartDashboard.putNumber("downPower", shootDownMotor.get());
  }
}
