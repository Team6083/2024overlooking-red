// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstants;

public class TransportSubsystem extends SubsystemBase {
  /** Creates a new TransportSubsystem. */
  private final VictorSPX trans;
  public TransportSubsystem() {
    trans = new VictorSPX(TransportConstants.kTrantsportChannel);
    trans.setInverted(true);
  }

  public void setTrans(){
    trans.set(VictorSPXControlMode.PercentOutput, 0.3);
  }

  public void setReTrans(){
    trans.set(VictorSPXControlMode.PercentOutput, -0.3);
  }

  public void stopMotor(){
    trans.set(VictorSPXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
