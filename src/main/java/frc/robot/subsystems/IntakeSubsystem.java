// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private final VictorSPX intakeMotor;

  public IntakeSubsystem() {
    intakeMotor = new VictorSPX(IntakeConstants.kIntakeChannel);
    intakeMotor.setInverted(IntakeConstants.kIntakeInverted);

  }

  public void setIntaking() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, IntakeConstants.kIntakePrecentage);
  }

  public void setThrowing() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, IntakeConstants.kThrowPrecentage);
  }

  public void stopMotor() {
    intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public double getIntakeMotorBusVoltage() {
    return intakeMotor.getBusVoltage();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.getNumber("IntakeMotorBusVoltage", getIntakeMotorBusVoltage());

  }
}
