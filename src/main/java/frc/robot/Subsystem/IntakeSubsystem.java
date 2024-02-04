// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;

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
    SmartDashboard.putNumber("setIntakeMotor1persentage", setIntakeMotor1age);
    SmartDashboard.putNumber("setIntakeMotor2persentage", setIntakeMotor2age);

  }

  public void setpersentage() {
    intakeMotor1.set(ControlMode.PercentOutput,setIntakeMotor1age);
    intakeMotor2.set(ControlMode.PercentOutput,setIntakeMotor2age);
  }

  public void stopMotor() {
    intakeMotor1.set(ControlMode.PercentOutput,0);
    intakeMotor2.set(ControlMode.PercentOutput,0);
  }

  public void setIntakecondition() {
    if (intakecondition) {
      setpersentage();
    } else {
      stopMotor();
    }
    intakecondition = !intakecondition;
  }

  public void getDashboard() {
    setIntakeMotor1age = SmartDashboard.getNumber("setIntakeMotor1presentage", 0.0);
    setIntakeMotor2age = SmartDashboard.getNumber("setIntakeMotor2persentage", 0.0);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    getDashboard();
  }
}
