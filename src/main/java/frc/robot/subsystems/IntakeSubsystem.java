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
  private final VictorSP intakeMotor1;
  private final VictorSP intakeMotor2;
  private final VictorSP trans;

  public IntakeSubsystem() {
    intakeMotor1 = new VictorSP(IntakeConstants.kIntakeUpChannel);
    intakeMotor2 = new VictorSP(IntakeConstants.kIntakeDownChannel);
    trans = new VictorSP(1);
    intakeMotor1.setInverted(IntakeConstants.kIntakeUpInverted);
    intakeMotor2.setInverted(IntakeConstants.kIntakeDownInverted);
  
  }

  public void setPrecentage() {
    intakeMotor1.set(IntakeConstants.kIntakePrecentage);
    intakeMotor2.set(IntakeConstants.kIntakePrecentage);
    trans.set(-0.2);
  }

  public void setTrans(){
    trans.set(-0.3);
  }

  public void stopMotor() {
    intakeMotor1.set(0);
    intakeMotor2.set(0);
    trans.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
