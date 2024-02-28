// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstants;

public class TransportSubsystem extends SubsystemBase {
  /** Creates a new TransportSubsystem. */
  private final CANSparkMax trans;
  private final Rev2mDistanceSensor dist;
  private final PowerDistributionSubsystem powerDistribution;

  public TransportSubsystem(PowerDistributionSubsystem powerDistribution) {

    trans = new CANSparkMax(TransportConstants.kTrantsportChannel, MotorType.kBrushless);
    trans.setInverted(TransportConstants.kTransportInverted);

    dist = new Rev2mDistanceSensor(Port.kOnboard);

    this.powerDistribution = powerDistribution;
  }

  public void setTrans() {
    setMotor(TransportConstants.kTransSpeed);
  }

  public void setReTrans() {
    setMotor(TransportConstants.kReTransSpeed);
  }

  public boolean isGetNote() {
    if (dist.isRangeValid()) {
      SmartDashboard.putNumber("Range dist", dist.getRange());
      SmartDashboard.putNumber("Timestamp dist", dist.getTimestamp());
    }
    return dist.getRange() <= TransportConstants.kDistRange;
  }

  public void stopMotor() {
    setMotor(0);
  }

  public void setMotor(double power) {
    if (powerDistribution.isTransportOverCurrent()) {
      stopMotor();
      return;
    }
    trans.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
