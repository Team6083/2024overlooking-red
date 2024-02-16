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
  private double positionOffset;  // 單駝峰

  public HookSubsystem() {
    line = new CANSparkMax(HookConstants.kHookLineChannel, MotorType.kBrushless);
    hookPID = new PIDController(HookConstants.kP, HookConstants.kI, HookConstants.kD);  // 常數用 kP kI kD ， hookPID
    lineEncoder = line.getEncoder();
    line.setInverted(HookConstants.kHookMotorInverted); // r

  }

  public void controlhook(double hookControlspeed) { // 單駝峰
    line.set(hookControlspeed);
    hookPID.setSetpoint(getHookposition());
  }

  public void controlmanul(double speed) { // 單駝峰 (首字母小寫)
    line.set(speed);
    hookPID.setSetpoint(getHookposition()); // setpoint不能直接這樣設，應該是要得hook的position
  }

  public double gethookSetpoint() { // 單駝峰
    return hookPID.getSetpoint();
  }

  public void setHookSetpoint(double setSetpoint) { // 單駝峰
    final double trueSetpoint = gethookSetpoint();  // true
    if (isPhylineExceed(trueSetpoint) != 0) {
      // hookPID.setSetpoint(
      //     (isPhylineExceed(trueSetpoint)) == 1 ? HookConstants.kHookPositionMax : HookConstants.kHookPositionMin);
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
    double linePower = hookPID.calculate(getHookposition()); // 單駝峰
    double modifiedLinepower = linePower; // 單駝峰
    if (Math.abs(modifiedLinepower) > HookConstants.kHookPower) {
      modifiedLinepower = HookConstants.kHookPower * (linePower > 0 ? 1 : 1); // kHookPower值是之後再調嗎
    }
    line.set(linePower);
    SmartDashboard.putNumber("linepower", modifiedLinepower);

  }

  public double getHookposition() {
    SmartDashboard.putNumber("", lineEncoder.getPosition());
    return (lineEncoder.getPosition()) + positionOffset; // positionoffset 現在沒有數值

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
