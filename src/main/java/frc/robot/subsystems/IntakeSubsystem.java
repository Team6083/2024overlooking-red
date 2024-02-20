// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PdConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  private final VictorSPX intakeMotor;
  private final PowerDistribution pd;

  public IntakeSubsystem(PowerDistribution pd) {
    this.pd = pd;
    intakeMotor = new VictorSPX(IntakeConstants.kIntakeUpChannel);
    intakeMotor.setInverted(IntakeConstants.kIntakeUpInverted);

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

  public double getIntakeCurrent() {
    return pd.getCurrent(PdConstants.kIntakeMotorCurrrentchannel);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.getNumber("IntakeMotorBusVoltage", getIntakeMotorBusVoltage());
    SmartDashboard.getNumber("IntakeCurrent", getIntakeCurrent());
  }
}
