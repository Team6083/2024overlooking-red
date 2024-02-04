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
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  private final VictorSPX shootUpMotor;
  private final VictorSPX shootDownMotor;
  private final Encoder upEncoder;
  private final Encoder downEncoder;
  private final PIDController upPidController;
  private final PIDController downPidController;

  private double inputUpMotorPercentage = 0.0;
  private double inputDownMotorPercentage = 0.0;
  private double rateToUpMotorPower =  0.0;
  private double rateToDownMotorPower = 0.0;
  private double controlUpRate = 0.0;
  private double controlDownRate = 0.0;

  public ShooterSubsystem() {

    shootUpMotor = new VictorSPX(ShooterConstants.kUpPWMID);
    shootDownMotor = new VictorSPX(ShooterConstants.kDownPWMID);

    upEncoder = new Encoder(0, 1);
    downEncoder = new Encoder(3, 4);

    upPidController = new PIDController(0.01, 0, 0);
    downPidController = new PIDController(0.01, 0, 0);

    shootUpMotor.setInverted(ShooterConstants.kUpMotorInvert);
    shootDownMotor.setInverted(ShooterConstants.kDownMotorInvert);

    SmartDashboard.putNumber("UpMotorPercentage", inputUpMotorPercentage);
    SmartDashboard.putNumber("DownMotorPercentage", inputDownMotorPercentage);
    SmartDashboard.putNumber("controlUpRate", controlUpRate);
    SmartDashboard.putNumber("controlDownRate", controlDownRate);

    upEncoder.reset();
    downEncoder.reset();
  }

  public void setPercentage() {
    shootUpMotor.set(ControlMode.PercentOutput, inputUpMotorPercentage);
    shootDownMotor.set(ControlMode.PercentOutput, inputDownMotorPercentage);
  }

  public void stopMotor() {
    shootUpMotor.set(ControlMode.PercentOutput, 0);
    shootDownMotor.set(ControlMode.PercentOutput, 0);
  }

  public void setPIDControl() {
    rateToUpMotorPower += upPidController.calculate(upEncoder.getRate(), controlUpRate);
    rateToDownMotorPower += downPidController.calculate(downEncoder.getRate(), controlDownRate);
    shootUpMotor.set(ControlMode.PercentOutput, rateToUpMotorPower);
    shootDownMotor.set(ControlMode.PercentOutput, rateToDownMotorPower);

  }

  public void getDashboard() {
    inputUpMotorPercentage = SmartDashboard.getNumber("UpMotorPercentage", 0.0);
    inputDownMotorPercentage = SmartDashboard.getNumber("DownMotorPercentage", 0.0);
    controlUpRate = SmartDashboard.getNumber("controlUpRate", 0.0);
    controlDownRate = SmartDashboard.getNumber("controlDownRate", 0.0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getDashboard();
    SmartDashboard.putNumber("upRate", upEncoder.getRate() / 2048.0);
    SmartDashboard.putNumber("downRate", downEncoder.getRate() / 2048.0);
  }
}
