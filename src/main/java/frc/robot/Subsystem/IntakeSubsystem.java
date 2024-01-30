// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private final VictorSP intakeMotor1;
  private final VictorSP intakeMotor2;

  public IntakeSubsystem() {
    intakeMotor1 = new VictorSP(IntakeConstants.kintakeonePWMID);
    intakeMotor2 = new VictorSP(IntakeConstants.kintaketwoPWMID);
    intakeMotor1.setInverted(IntakeConstants.kintakeoneInvert);
    intakeMotor2.setInverted(IntakeConstants.kintaketwoInvert);
  

  }

  public void setVoltage() {
    intakeMotor1.setVoltage(4);
    intakeMotor2.setVoltage(4);
  }

  public void stopVoltage(){
    intakeMotor1.setVoltage(0);
    intakeMotor2.setVoltage(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  } 
}
