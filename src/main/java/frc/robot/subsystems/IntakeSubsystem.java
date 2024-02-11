// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private final VictorSP intakeUpMotor;
  private final VictorSP intakeDownMotor;

  public IntakeSubsystem() {
    intakeUpMotor = new VictorSP(IntakeConstants.kIntakeUpChannel);
    intakeDownMotor = new VictorSP(IntakeConstants.kIntakeDownChannel);
    intakeUpMotor.setInverted(IntakeConstants.kIntakeUpInverted);
    intakeDownMotor.setInverted(IntakeConstants.kIntakeDownInverted);
  
  }

  public void setIntaking() {
    intakeUpMotor.set(IntakeConstants.kIntakePrecentage);
    intakeDownMotor.set(IntakeConstants.kIntakePrecentage);
  }

  public void setThrowing(){
    intakeUpMotor.set(IntakeConstants.kThrowPrecentage);
    intakeDownMotor.set(IntakeConstants.kThrowPrecentage);
  }

  public void stopMotor() {
    intakeUpMotor.set(0);
    intakeDownMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
