// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.HookConstants;

public class HookSubsystem extends SubsystemBase {
  /** Creates a new HookSubsystem. */
  private final PIDController hookPID;
  private final CANSparkMax line;
  private final RelativeEncoder lineEncoder;
  private double positionOffset;

  public HookSubsystem() {
    line = new CANSparkMax(HookConstants.kHookLineChannel, MotorType.kBrushless);
    hookPID = new PIDController(HookConstants.kP, HookConstants.kI, HookConstants.kD);
    lineEncoder = line.getEncoder();
    line.setInverted(HookConstants.kHookMotorInverted);

  }

  public void controlHook(double hookControlSpeed) {
    line.set(hookControlSpeed);
    hookPID.setSetpoint(getHookPosition());
  }

  public void controlManul(double speed) {
    line.set(speed);
    hookPID.setSetpoint(getHookPosition());
  }

  public double getHookSetpoint() {
    return hookPID.getSetpoint();
  }

  public void setHookSetpoint(double setSetpoint) {
    final double trueSetpoint = getHookSetpoint();
    if (isPhylineExceed(trueSetpoint) != 0) {
      hookPID.setSetpoint(
          (isPhylineExceed(trueSetpoint)) == 1 ? HookConstants.kHookPositionMax : HookConstants.kHookPositionMin);
      return;
    }
    // setSetpoint += trueSetpoint;  // 這樣setpoint最後的數值就不是當初放進函式的，我不太理解為甚麼要判斷trueSetpoint的值以及把setSetpoint的值與trueSetpoint相加
    if (isPhylineExceed(setSetpoint) == -1) {
      setSetpoint = HookConstants.kHookPositionMin;
    } else if (isPhylineExceed(setSetpoint) == 1) {
      setSetpoint = HookConstants.kHookPositionMax;
    }

    hookPID.setSetpoint(setSetpoint);

  }

  public void PIDControl() {
    double linePower = hookPID.calculate(getHookPosition());
    double modifiedLinePower = linePower;
    if (Math.abs(modifiedLinePower) > HookConstants.kHookPower) {
      modifiedLinePower = HookConstants.kHookPower * (linePower > 0 ? 1 : 1); // TO DO -> kHookPower
    }
    line.set(linePower);
    SmartDashboard.putNumber("linepower", modifiedLinePower);

  }

  public double getHookPosition() {
    SmartDashboard.putNumber("", lineEncoder.getPosition());
    return (lineEncoder.getPosition()) + positionOffset; // TO DO -> positionOffset

  }

  public void stopMotor() {
    line.set(0.0);
  }

  public void resetHookEncoder() {
    lineEncoder.setPosition(0);
  }

  private int isPhylineExceed(double position) {
    return (position < HookConstants.kHookPositionMin ? -1 : (position > HookConstants.kHookPositionMax) ? 1 : 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("HookPID", hookPID);
  }
}
