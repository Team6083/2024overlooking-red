// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstants;

public class TransportSubsystem extends SubsystemBase {
  /** Creates a new TransportSubsystem. */
  private final VictorSPX trans;
  private final Rev2mDistanceSensor dist;

  public TransportSubsystem() {

    trans = new VictorSPX(TransportConstants.kTrantsportChannel);
    trans.setInverted(true);

    dist = new Rev2mDistanceSensor(Port.kOnboard);
  }

  public void setTrans() {
    trans.set(VictorSPXControlMode.PercentOutput, 0.3);//value變成constants
  }

  public void setReTrans() {
    trans.set(VictorSPXControlMode.PercentOutput, -0.3);//同上
  }

  public void intakeTrans(){
    if(dist.isRangeValid()){
      SmartDashboard.putNumber("Range dist", dist.getRange());
      SmartDashboard.putNumber("Timestamp dist", dist.getTimestamp());
    }
    if(dist.getRange()<=8.0){//同上
      stopMotor();
    }else{
      setTrans();
    }
  }

  public void stopMotor() {
    trans.set(VictorSPXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
