// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private final VictorSPX intakeMotor;
  private final PowerDistributionSubsystem powerDistributionSubsystem;
  private final boolean isGetNote;

  public IntakeSubsystem(PowerDistributionSubsystem powerDistributionSubsystem ,boolean isGetNote) {
    this.powerDistributionSubsystem = powerDistributionSubsystem;
    this.isGetNote = isGetNote;
    intakeMotor = new VictorSPX(IntakeConstants.kIntakeChannel);
    intakeMotor.setInverted(IntakeConstants.kIntakeInverted);

  }

  public Command runIntake() {
    Command runIntake = Commands.startEnd(() -> setIntaking(), () -> stopMotor());
    Command reIntake = Commands.startEnd(() -> setThrowing(), () -> stopMotor());
    runIntake.setName(getName());
    reIntake.setName(getName());
    return runIntake();
  }

  public void setIntaking() {
    setMotor(IntakeConstants.kIntakeVoltage);
  }

  public void setThrowing() {
    setMotor(IntakeConstants.kThrowPrecentage);
  }

  public void stopMotor() {
    setMotor(0);
  }

  public void setMotor(double voltage) {
    if (powerDistributionSubsystem.isIntakeOverCurrent()) {
      stopMotor();
      return;
    }
    intakeMotor.set(VictorSPXControlMode.PercentOutput, voltage / getIntakeMotorBusVoltage());
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
