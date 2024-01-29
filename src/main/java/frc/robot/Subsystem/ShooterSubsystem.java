// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  private final VictorSP shootUpMotor;
  private final VictorSP shootDownMotor;
  private double UpMotorVoltage = 0.0;
  private double DownMotorVoltage = 0.0;
  private Boolean shootCondition = true;

  public ShooterSubsystem() {
    shootUpMotor = new VictorSP(ShooterConstants.kUpPWMID);
    shootDownMotor = new VictorSP(ShooterConstants.kDownPWMID);
    shootUpMotor.setInverted(ShooterConstants.kUpMotorInvert);
    shootDownMotor.setInverted(ShooterConstants.kDownMotorInvert);
    SmartDashboard.putNumber("UpMotorVoltage", UpMotorVoltage);
    SmartDashboard.putNumber("DownMotorVoltage", DownMotorVoltage);
  }

  public void setVoltage() {
    shootUpMotor.setVoltage(SmartDashboard.getNumber("UpMotorVoltage", 0));
    shootDownMotor.setVoltage(SmartDashboard.getNumber("DownMotorVoltage", 0));
  }

  public void stopMotor() {
    shootUpMotor.setVoltage(0.0);
    shootDownMotor.setVoltage(0.0);
  }

  public void setShooterCondition() {
    if (shootCondition) {
      setVoltage();
    } else {
      stopMotor();
    }
    shootCondition = !shootCondition;
  }

  // public void getDashboard() {
  //   SmartDashboard.getNumber("UpMotorVoltage", 0.0);
  //   SmartDashboard.getNumber("DownMotorVoltage", 0.0);
  // }

  public void putDashboard() {

    SmartDashboard.putNumber("Output_UpVoltage", shootUpMotor.get() * 12.0);
    SmartDashboard.putNumber("Output_DownVoltage", shootDownMotor.get() * 12.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    putDashboard();
    // getDashboard();
  }
}
