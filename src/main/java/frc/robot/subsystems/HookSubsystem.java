// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HookConstants;

public class HookSubsystem extends SubsystemBase {
  /** Creates a new HookSubsystem. */
  private final PIDController HookPID;
  private final CANSparkMax line;
  private final RelativeEncoder lineEncoder;
  private double positionoffset;

  public HookSubsystem() {
    line = new CANSparkMax(HookConstants.kHookLineChannel, MotorType.kBrushless);
    HookPID = new PIDController(HookConstants.HP, HookConstants.HI, HookConstants.HD);
    lineEncoder = line.getEncoder();
    line.setInverted(HookConstants.kHookMotoInverted);

  }

  public void controlhook(double hookcontrolspeed) {
    line.set(hookcontrolspeed);
    HookPID.setSetpoint(getHookposition());
  }

  public void controlmanul(double riseSpeed) {
    line.set(riseSpeed);
    HookPID.setSetpoint(HookConstants.kInitSetpoint);
  }

  public double gethooksetpoint() {
    return HookPID.getSetpoint();
  }

  public void setHooksetpoint(double setSetpoint) {
    final double tureSetpoint = gethooksetpoint();
    if (islineExceed(tureSetpoint) != 0) {
      HookPID.setSetpoint(
          (islineExceed(tureSetpoint)) == 1 ? HookConstants.kHookPositionMax : HookConstants.kHookPositionMin);
      return;
    }
    setSetpoint += tureSetpoint;
    if (islineExceed(setSetpoint) == -1) {
      setSetpoint = HookConstants.kHookPositionMin;
    } else if (islineExceed(setSetpoint) == 1) {
      setSetpoint = HookConstants.kHookPositionMax;
    }

    HookPID.setSetpoint(setSetpoint);

  }

  public void PIDControl() {
    double linepower = HookPID.calculate(getHookposition());
    double modifiedlinepower = linepower;
    if (Math.abs(modifiedlinepower) > HookConstants.kHookPower) {
      modifiedlinepower = HookConstants.kHookPower * (linepower > 0 ? 1 : 1);
    }
    line.set(linepower);
    SmartDashboard.putNumber("linepower", modifiedlinepower);

  }

  public double getHookposition() {
    SmartDashboard.putNumber("", lineEncoder.getPosition());
    return (lineEncoder.getPosition()) + positionoffset;

  }

  public void stopMotor() {
    line.set(0.0);
  }

  public void resetHookEncoder() {
    lineEncoder.setPosition(0);
  }

  private int islineExceed(double position) {
    return (position < HookConstants.kHookPositionMin ? -1 : (position > HookConstants.kHookPositionMax) ? 1 : 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  SmartDashboard.putData("HookPID", HookPID);
  }
}
