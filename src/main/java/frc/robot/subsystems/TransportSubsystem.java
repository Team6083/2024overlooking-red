// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstants;

public class TransportSubsystem extends SubsystemBase {
  /** Creates a new TransportSubsystem. */
  private final CANSparkMax transportMotor;
  private final Rev2mDistanceSensor distanceSensor;
  private final PowerDistributionSubsystem powerDistributionSubsystem;

  public TransportSubsystem(PowerDistributionSubsystem powerDistribution) {

    transportMotor = new CANSparkMax(TransportConstants.kTransportChannel, MotorType.kBrushless);
    transportMotor.setInverted(TransportConstants.kTransportInverted);
    distanceSensor = new Rev2mDistanceSensor(Port.kOnboard);

    this.powerDistributionSubsystem = powerDistribution;
  }

  public Command ReTransportIntakeCmd() {
    return this.startEnd(() -> this.setReTransport(), () -> this.stopMotor());
  }

  public Command transportIntakeCmd() {
    return this.startEnd(() -> this.setTransport(), () -> this.stopMotor());
  }

  public Command TrasportShooter() {
    return this.startEnd(() -> this.setTransport(), () -> this.stopMotor());
  }

  public void setTransport() {
    setMotor(TransportConstants.kTransVoltage);
  }

  public void setReTransport() {
    setMotor(TransportConstants.kReTransVoltage);
  }

  public boolean isGetNote() {
    if (distanceSensor.isRangeValid()) {
      // SmartDashboard.putNumber("Range dist", distanceSensor.getRange());
      // SmartDashboard.putNumber("Timestamp dist", distanceSensor.getTimestamp());
      return distanceSensor.getRange() <= TransportConstants.kDistanceRange;
    }
    return false;
  }

  public void stopMotor() {
    transportMotor.set(0);
  }

  public void setMotor(double voltage) {
    if (powerDistributionSubsystem.isTransportOverCurrent()) {
      stopMotor();
      return;
    }
    transportMotor.setVoltage(voltage);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType(" TransportSubsystem");
    builder.addDoubleProperty("Range dist", () -> distanceSensor.getRange(), null);
    builder.addDoubleProperty("Timestamp dist", () -> distanceSensor.getTimestamp(), null);
  }
}
