// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private final VictorSPX intakeMotor1;
  private final VictorSPX intakeMotor2;

  public IntakeSubsystem() {
    intakeMotor1 = new VictorSPX(IntakeConstants.kintakeonePWMID);
    intakeMotor2 = new VictorSPX(IntakeConstants.kintaketwoPWMID);
    intakeMotor2.follow(intakeMotor1);
  
  }

  public void setpersentage() {
    intakeMotor1.set(ControlMode.PercentOutput,0.5);
  }

  public void stopMotor() {
    intakeMotor1.set(ControlMode.PercentOutput,0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
