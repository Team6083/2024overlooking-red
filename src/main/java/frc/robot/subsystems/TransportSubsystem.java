// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.*;
import com.revrobotics.Rev2mDistanceSensor.Port;

public class TransportSubsystem extends SubsystemBase {

  /** Creates a new TransportSubsystem. */
  private final VictorSPX trans;

  private Rev2mDistanceSensor dist;

  public TransportSubsystem() {
    trans = new VictorSPX(TransportConstants.kTrantsportChannel);
    trans.setInverted(true);
    dist = new Rev2mDistanceSensor(Port.kOnboard);
    dist.setAutomaticMode(true);
  }

  public void setTrans() {
    trans.set(VictorSPXControlMode.PercentOutput, 0.5);
  }

  public void setReTrans() {
    trans.set(VictorSPXControlMode.PercentOutput, -0.3);
  }

  public void stopMotor() {
    trans.set(VictorSPXControlMode.PercentOutput, 0);
  }

  public void distanceSensor() {
    if (dist.isRangeValid()) {
      SmartDashboard.putNumber("Range dist", dist.getRange());
      SmartDashboard.putNumber("Timestamp dist", dist.getTimestamp());
    }   
    if(dist.getRange()<=8){
      trans.set(VictorSPXControlMode.PercentOutput, 0);
  }
    }

  

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
