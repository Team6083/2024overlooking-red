// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private final VictorSPX intakeUpMotor;
  private final VictorSPX intakeDownMotor;

  public IntakeSubsystem() {
    intakeUpMotor = new VictorSPX(IntakeConstants.kIntakeUpChannel);
    intakeDownMotor = new VictorSPX(IntakeConstants.kIntakeDownChannel);
    intakeUpMotor.setInverted(IntakeConstants.kIntakeUpInverted);
    intakeDownMotor.setInverted(IntakeConstants.kIntakeDownInverted);
  
  }

  public void setIntaking() {
    intakeUpMotor.set(VictorSPXControlMode.PercentOutput, IntakeConstants.kIntakePrecentage);
    intakeDownMotor.set(VictorSPXControlMode.PercentOutput, IntakeConstants.kIntakePrecentage);
  }

  public void setThrowing(){
    intakeUpMotor.set(VictorSPXControlMode.PercentOutput, IntakeConstants.kThrowPrecentage);
    intakeDownMotor.set(VictorSPXControlMode.PercentOutput, IntakeConstants.kThrowPrecentage);
  }

  public void stopMotor() {
    intakeUpMotor.set(VictorSPXControlMode.PercentOutput,0);
    intakeDownMotor.set(VictorSPXControlMode.PercentOutput,0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
