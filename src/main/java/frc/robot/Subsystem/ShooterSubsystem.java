// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RiseShooterConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  private final VictorSPX shootUpMotor;
  private final VictorSPX shootDownMotor;

  private double UpMotorPercentage = 0.0;
  private double DownMotorPercentage = 0.0;
  private Boolean shootCondition = true;

  public ShooterSubsystem() {
    shootUpMotor = new VictorSPX(ShooterConstants.kUpPWMID);
    shootDownMotor = new VictorSPX(ShooterConstants.kDownPWMID);
    shootUpMotor.setInverted(ShooterConstants.kUpMotorInvert);
    shootDownMotor.setInverted(ShooterConstants.kDownMotorInvert);
    SmartDashboard.putNumber("UpMotorPercentage", UpMotorPercentage);
    SmartDashboard.putNumber("DownMotorPercentage", DownMotorPercentage);
  }

  public void setPercentage() {
    shootUpMotor.set(ControlMode.PercentOutput,UpMotorPercentage );
    shootDownMotor.set(ControlMode.PercentOutput,DownMotorPercentage );}

  public void stopMotor() {
    shootUpMotor.set(ControlMode.PercentOutput, 0);
    shootDownMotor.set(ControlMode.PercentOutput,0 );
  }

  public void setShooterCondition() {
    if (shootCondition) {
      setPercentage();
    } else {
      stopMotor();
    }
    shootCondition = !shootCondition;
  }

  public void getDashboard() {
    UpMotorPercentage = SmartDashboard.getNumber("UpMotorPercentage", 0.0);
    DownMotorPercentage = SmartDashboard.getNumber("DownMotorPercentage", 0.0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getDashboard();
  }
}
