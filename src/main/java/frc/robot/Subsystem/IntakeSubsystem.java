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
  private double setIntakeMotor1age = 0.0;
  private double setIntakeMotor2age = 0.0;
  private boolean intakecondition = true;

  public IntakeSubsystem() {
    intakeMotor1 = new VictorSP(IntakeConstants.kintakeonePWMID);
    intakeMotor2 = new VictorSP(IntakeConstants.kintaketwoPWMID);
    intakeMotor1.setInverted(IntakeConstants.kintakeoneInvert);
    intakeMotor2.setInverted(IntakeConstants.kintaketwoInvert);
    SmartDashboard.putNumber("setIntakeMotor1Voltage", setIntakeMotor1age);
    SmartDashboard.putNumber("setIntakeMotor2Voltage", setIntakeMotor2age);

  }

  public void setVoltage() {
    intakeMotor1.setVoltage(SmartDashboard.getNumber("setIntakeMotor1Voltage", 0));
    intakeMotor2.setVoltage(SmartDashboard.getNumber("setIntakeMotor2Voltage", 0));
  }

  public void stopVoltage() {
    intakeMotor1.setVoltage(0);
    intakeMotor2.setVoltage(0);
  }

  public void setIntakecondition() {
    if (intakecondition) {
      setVoltage();
    } else {
      stopVoltage();
    }
    intakecondition = !intakecondition;
  }

  public void getDashboard() {
    SmartDashboard.getNumber("setIntakeMotor1Voltage", 0.0);
    SmartDashboard.getNumber("setIntakeMotor2Voltage", 0.0);

  }

  public void putDashboard() {
    SmartDashboard.putNumber("out_putsetIntakeMotor1Voltage", intakeMotor1.get() * 12.0);
    SmartDashboard.putNumber("out_putsetIntakeMotor2Voltage", intakeMotor2.get() * 12.0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    putDashboard();
    getDashboard();
  }
}
