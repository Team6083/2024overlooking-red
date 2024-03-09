// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RotateIntakeConstants;

public class RotateIntakeSubsystem extends SubsystemBase {
  /** Creates a new RotateIntake. */
  private final VictorSPX rotateIntake;
  private final DutyCycleEncoder rotateEncoder;
  private final PowerDistributionSubsystem powerDistributionSubsystem;
  private final PIDController pid;
  private final double initDegree = 90.0;

  public RotateIntakeSubsystem(PowerDistributionSubsystem powerDistributionSubsystem) {
    rotateIntake = new VictorSPX(RotateIntakeConstants.kRotateIntakeChannel);
    rotateIntake.setInverted(RotateIntakeConstants.kRotateIntakeInverted);
    rotateIntake.setNeutralMode(NeutralMode.Brake);
    rotateEncoder = new DutyCycleEncoder(RotateIntakeConstants.kRotateEncoderChannel);
    this.powerDistributionSubsystem = powerDistributionSubsystem;
    pid = new PIDController(RotateIntakeConstants.kP, RotateIntakeConstants.kI, RotateIntakeConstants.kD);
    pid.setSetpoint(initDegree);
  }

  public void upIntake() {
    pid.setSetpoint(RotateIntakeConstants.kUpDegree);
  }

  public void downIntake() {
    pid.setSetpoint(RotateIntakeConstants.kDownDegree);
  }

  public void initIntake() {
    pid.setSetpoint(initDegree);
  }

  public void setPID() {
    double rotateVoltage = pid.calculate(getAngleDegree());
    double modifiedRotateVoltage = rotateVoltage;
    if (Math.abs(modifiedRotateVoltage) > RotateIntakeConstants.kRotateVoltLimit) {
      modifiedRotateVoltage = RotateIntakeConstants.kRotateVoltLimit * (rotateVoltage > 0 ? 1 : -1);
    }
    setMotor(modifiedRotateVoltage);
  }

  public double getAngleDegree() {
    double degree = (RotateIntakeConstants.kEncoderInverted ? -1.0 : 1.0)
        * ((rotateEncoder.getAbsolutePosition() * 360.0) - 189.0);
    SmartDashboard.putNumber("rotateIntakeDegree", degree);
    return degree;
  }

  public void setMotor(double voltage) {
    if (powerDistributionSubsystem.isRotateIntakeOverCurrent()) {
      stopMotor();
    }
    rotateIntake.set(VictorSPXControlMode.PercentOutput, voltage / getBusVoltage());
  }

  public void stopMotor() {
    rotateIntake.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  public double getBusVoltage() {
    return rotateIntake.getBusVoltage();
  }

  public Command upIntakeCmd() {
    Command upCmd = Commands.runOnce(() -> upIntake(), this);
    upCmd.setName("upIntakeCmd");
    return upCmd;
  }

  public Command downIntakeCmd() {
    Command downCmd = Commands.runOnce(() -> downIntake(), this);
    downCmd.setName("downIntakeCmd");
    return downCmd;
  }

  public Command initIntakeCmd() {
    Command initCmd = Commands.runOnce(() -> initIntake(), this);
    initCmd.setName("initIntakeCmd");
    return initCmd;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setPID();
  }
}
