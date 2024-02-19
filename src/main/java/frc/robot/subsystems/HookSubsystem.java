// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HookConstants;

public class HookSubsystem extends SubsystemBase {
  /** Creates a new HookSubsystem. */
  private final PIDController linePID;
  private final PIDController hookMotorPID;
  private final CANSparkMax line;
  public final VictorSPX hookMotor1;
  public final VictorSPX hookMotor2;
  private final RelativeEncoder lineEncoder;
  private double positionOffset=0.0;

  public HookSubsystem() {
    line = new CANSparkMax(HookConstants.kHookLineChannel, MotorType.kBrushless);
    hookMotor1 = new VictorSPX(HookConstants.kHookMotor1Cnannel);
    hookMotor2 = new VictorSPX(HookConstants.kHookMotor2Cnannel);
    linePID = new PIDController(HookConstants.kP, HookConstants.kI, HookConstants.kD);
    hookMotorPID = new PIDController(HookConstants.kP, HookConstants.kI, HookConstants.kD);
    lineEncoder = line.getEncoder();// 這邊忘記設定lineEncoder的PositionConversionFactor，這你算出來的值一定不會是正確的喔，你可以查查要怎麼設定
    lineEncoder.setPositionConversionFactor(HookConstants.kHookPositionConversionfactor);
    line.setInverted(HookConstants.kHookMotor1Inverted);

  }

  public void controlLine(double hookControlSpeed) {
    line.set(hookControlSpeed);
    linePID.setSetpoint(getHookPosition());
  }

  public void controlHookMotor(double speed) {
    hookMotor1.set(VictorSPXControlMode.PercentOutput, HookConstants.khookmotor1Power);
    hookMotorPID.setSetpoint(getHookPosition());
  }

  public double getLineSetpoint() {
    return linePID.getSetpoint();
  }

  public double getHookMotorSetpoint() {
    return hookMotorPID.getSetpoint();
  }

  public void setLineSetpoint(double setSetpoint) {
   final double currentSetpoint = setSetpoint;

    if (isPhyLineExceed(currentSetpoint) != 0) {
      linePID.setSetpoint(
          (isPhyLineExceed(currentSetpoint)) >= 1 ? HookConstants.kHookPositionMax : HookConstants.kHookPositionMin);
      return;
    }

    linePID.setSetpoint(currentSetpoint);

  }

  public void setHookMotorsetpoint(double motorSetpoint) {
    final double currentSetpoint = motorSetpoint;
    if (isPhyLineExceed(currentSetpoint) != 0) {
      hookMotorPID.setSetpoint(
          (isPhyLineExceed(currentSetpoint)) == 1 ? HookConstants.kHookPositionMax : HookConstants.kHookPositionMin);
      return;
    }
    hookMotorPID.setSetpoint(currentSetpoint);

  }

  public void linePIDControl() {
    double linePower = linePID.calculate(lineEncoder.getPosition(), getLineSetpoint());
    double modifiedLinePower = linePower;
    if (Math.abs(modifiedLinePower) > HookConstants.kHookPower) {
      modifiedLinePower = HookConstants.kHookPower * (linePower > 0 ? 1 :-1);
    }
    line.setVoltage(linePower);
    SmartDashboard.putNumber("linepower", modifiedLinePower);

  }

  public void hookMotorPIDControl() {
    double hookMotor1Power = hookMotorPID.calculate(lineEncoder.getPosition(),getHookMotorSetpoint());
    double modifiedHookMotor1Power = hookMotor1Power;
    if (Math.abs(modifiedHookMotor1Power) > HookConstants.kHookPower) {
      modifiedHookMotor1Power = HookConstants.kHookPower * (hookMotor1Power > 0 ? 1 :-1);
    }
     hookMotor1.set(VictorSPXControlMode.PercentOutput,hookMotor1Power/12);
    SmartDashboard.putNumber("hookmotorpower", modifiedHookMotor1Power);

  }

  public double getHookPosition() {
    SmartDashboard.putNumber("Position", lineEncoder.getPosition());
    return (lineEncoder.getPosition()) + positionOffset;

  }

  public void stopMotor() {
    line.set(0.0);
    hookMotor1.set(VictorSPXControlMode.PercentOutput, 0.0);
    hookMotor2.follow(hookMotor1);
  }

  public void resetHookEncoder() {
    lineEncoder.setPosition(0);
  }

  private int isPhyLineExceed(double position) {
    return (position < HookConstants.kHookPositionMin ? -1 : (position > HookConstants.kHookPositionMax) ? 1 : 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("LINEPID", linePID);
    SmartDashboard.putData("hook motor", hookMotorPID);
  }
}
