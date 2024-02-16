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
  private final PIDController HookPID;
  private final CANSparkMax line;
  private final RelativeEncoder lineEncoder;
  private double positionoffset;  // 單駝峰

  public HookSubsystem() {
    line = new CANSparkMax(HookConstants.kHookLineChannel, MotorType.kBrushless);
    HookPID = new PIDController(HookConstants.HP, HookConstants.HI, HookConstants.HD);  // 常數用 kP kI kD ， hookPID
    lineEncoder = line.getEncoder();
    line.setInverted(HookConstants.kHookMotoInverted); // 少一個 r

  }

  public void controlhook(double hookcontrolspeed) { // 單駝峰
    line.set(hookcontrolspeed);
    HookPID.setSetpoint(getHookposition());
  }

  public void controlmanul(double Speed) { // 單駝峰 (首字母小寫)
    line.set(Speed);
    HookPID.setSetpoint(HookConstants.kInitSetpoint); // setpoint不能直接這樣設，應該是要得hook的position
  }

  public double gethooksetpoint() { // 單駝峰
    return HookPID.getSetpoint();
  }

  public void setHooksetpoint(double setSetpoint) { // 單駝峰
    final double tureSetpoint = gethooksetpoint();  // true
    if (isPhylineExceed(tureSetpoint) != 0) {
      HookPID.setSetpoint(
          (isPhylineExceed(tureSetpoint)) == 1 ? HookConstants.kHookPositionMax : HookConstants.kHookPositionMin);
      return;
    }
    setSetpoint += tureSetpoint;  // 這樣setpoint最後的數值就不是當初放進函式的，我不太理解為甚麼要判斷trueSetpoint的值以及把setSetpoint的值與trueSetpoint相加
    if (isPhylineExceed(setSetpoint) == -1) {
      setSetpoint = HookConstants.kHookPositionMin;
    } else if (isPhylineExceed(setSetpoint) == 1) {
      setSetpoint = HookConstants.kHookPositionMax;
    }

    HookPID.setSetpoint(setSetpoint);

  }

  public void PIDControl() {
    double linepower = HookPID.calculate(getHookposition()); // 單駝峰
    double modifiedlinepower = linepower; // 單駝峰
    if (Math.abs(modifiedlinepower) > HookConstants.kHookPower) {
      modifiedlinepower = HookConstants.kHookPower * (linepower > 0 ? 1 : 1); // kHookPower值是之後再調嗎
    }
    line.set(linepower);
    SmartDashboard.putNumber("linepower", modifiedlinepower);

  }

  public double getHookposition() {
    SmartDashboard.putNumber("", lineEncoder.getPosition());
    return (lineEncoder.getPosition()) + positionoffset; // positionoffset 現在沒有數值

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
    SmartDashboard.putData("HookPID", HookPID);
  }
}
