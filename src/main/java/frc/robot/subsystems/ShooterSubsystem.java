// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.can.VictorSPX;
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
  private final PIDController upPidController;
  private final PIDController downPidController;
  private final SimpleMotorFeedforward rateControl;

  public ShooterSubsystem() {

    shootUpMotor = new VictorSP(ShooterConstants.kShooterUpChannel);
    shootDownMotor = new VictorSP(ShooterConstants.kShooterDownChannel);

    upEncoder = new Encoder(0, 1);
    downEncoder = new Encoder(2, 3);

    upPidController = new PIDController(0.002, 0, 0);
    downPidController = new PIDController(0.002, 0, 0);

    shootUpMotor.setInverted(ShooterConstants.kUpMotorInverted);
    shootDownMotor.setInverted(ShooterConstants.kDownMotorInverted);

    rateControl = new SimpleMotorFeedforward(0, 0.23, 0.71);

    resetEncoder();
    SmartDashboard.putNumber("shooter_rate", 0);
    SmartDashboard.putNumber("shooter_Voltage", 0);
  }

  public void setManualPercentage() {
    double voltage = 10;
    shootUpMotor.setVoltage(voltage);
    shootDownMotor.setVoltage(voltage);
  }

  public void resetEncoder() {
    upEncoder.reset();
    downEncoder.reset();
  }

  public void setPIDPercentage(double distance) {
    shootUpMotor.set(distance);
    shootDownMotor.set(distance);
  }

  public void setSetpoint(double distance) {
    upPidController.setSetpoint(distance);
    downPidController.setSetpoint(distance);
  }

  public void setTestRate() {
    double rate = SmartDashboard.getNumber("shooter_rate", 0.0);
    rate = rateControl.calculate(rate);
    shootUpMotor.setVoltage(rate);
    shootDownMotor.setVoltage(rate);
  }

  public void setPIDRate() {
    final double rateToUpMotorPower = upPidController.calculate(-upEncoder.getRate()/2048.0);
    final double rateToDownMotorPower = downPidController.calculate(-downEncoder.getRate()/2048.0);
    shootUpMotor.set(rateToUpMotorPower);
    shootDownMotor.set(rateToDownMotorPower);
    SmartDashboard.putNumber("rateToUpMotorPower", rateToUpMotorPower);
    SmartDashboard.putNumber("rateToDownMotorPower", rateToDownMotorPower);
  }

  public void stopMotor() {
    shootUpMotor.set(0);
    shootDownMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("upRate", -upEncoder.getRate() / 2048.0);
    SmartDashboard.putNumber("downRate", -downEncoder.getRate() / 2048.0);
    SmartDashboard.putNumber("upPower", shootUpMotor.get());
    SmartDashboard.putNumber("downPower", shootDownMotor.get());
  }
}
