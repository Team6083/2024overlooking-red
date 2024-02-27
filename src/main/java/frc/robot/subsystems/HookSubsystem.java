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
  public final VictorSPX hookLeftMotor;
  public final VictorSPX hookRightMotor;
  private final RelativeEncoder lineEncoder;
  private double positionOffset = 0.0;

  public HookSubsystem() {
    line = new CANSparkMax(HookConstants.kHookLineChannel, MotorType.kBrushless);
    hookLeftMotor = new VictorSPX(HookConstants.kHookLeftMotorCnannel);
    hookRightMotor = new VictorSPX(HookConstants.kHookRightMotorCnannel);
    linePID = new PIDController(HookConstants.kP, HookConstants.kI, HookConstants.kD);
    hookMotorPID = new PIDController(HookConstants.kP, HookConstants.kI, HookConstants.kD);
    lineEncoder = line.getEncoder();
    lineEncoder.setPositionConversionFactor(HookConstants.kHookPositionConversionfactor);
    line.setInverted(HookConstants.kHookMotorLeftInverted);
  }

  public void controlLine(double hookControlSpeed) {
    setLineMotor(hookControlSpeed);
    linePID.setSetpoint(getHookPosition());
  }

  public void controlHookMotor(double speed) {
    setLeftMotor(HookConstants.kHookMotorLeftPower);
    setRightMotor(HookConstants.kHookMotorRightPower);
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
    if (Math.abs(modifiedLinePower) > HookConstants.kLinePower) {
      modifiedLinePower = HookConstants.kLinePower * (linePower > 0 ? 1 : -1);
    }
    line.setVoltage(linePower);
    SmartDashboard.putNumber("linepower", modifiedLinePower);

  }

  public void hookLeftMotorPIDContro() {
    double hookMotorLeftPower = hookMotorPID.calculate(lineEncoder.getPosition(), getHookMotorSetpoint());
    double modifiedHookMotorLeftPower = hookMotorLeftPower;
    if (Math.abs(modifiedHookMotorLeftPower) > HookConstants.kHookMotorLeftPower) {
      modifiedHookMotorLeftPower = HookConstants.kHookMotorLeftPower * (hookMotorLeftPower > 0 ? 1 : -1);
    }
    setLeftMotor(hookMotorLeftPower / getHookLeftMotorBusVoltage());
    SmartDashboard.putNumber("hookmotor1power", modifiedHookMotorLeftPower);

    double hookMotorRightPower = hookMotorPID.calculate(lineEncoder.getPosition(), getHookMotorSetpoint());
    double modifiedHookMotorRightPower = hookMotorLeftPower;
    if (Math.abs(modifiedHookMotorRightPower) > HookConstants.kHookMotorRightPower) {
      modifiedHookMotorRightPower = HookConstants.kHookMotorRightPower * (hookMotorRightPower > 0 ? 1 : -1);
    }
    setRightMotor(hookMotorRightPower / getHookRightMotorBusVoltage());
    SmartDashboard.putNumber("hookmotor2power", modifiedHookMotorRightPower);
  }

  public double getHookPosition() {
    SmartDashboard.putNumber("Position", lineEncoder.getPosition());
    return (lineEncoder.getPosition()) + positionOffset;

  }

  public double getHookLeftMotorBusVoltage(){
    return hookLeftMotor.getBusVoltage();
  }

  public double getHookRightMotorBusVoltage(){
    return hookRightMotor.getBusVoltage();
  }

  public void stopLineMotor() {
    setLineMotor(0.0);
  }

  public void stopHookLeftMotor() {
    setLeftMotor(0.0);
  }

  public void stopHookRightMotor() {
    setRightMotor(0.0);
  }

  public void setLeftMotor(double power) {
    if (PowerDistributionSubsystem.isHookLeftOverCurrent()) {
      stopHookLeftMotor();
      return;
    }
    hookLeftMotor.set(VictorSPXControlMode.PercentOutput, power);
  }

  public void setRightMotor(double power) {
    if (PowerDistributionSubsystem.isHookRightOverCurrent()) {
      stopHookRightMotor();
      return;
    }
    hookRightMotor.set(VictorSPXControlMode.PercentOutput, power);
  }

  public void setLineMotor(double power) {
    if (PowerDistributionSubsystem.isLineMoterOverCurrent()) {
      stopLineMotor();
      return;
    }
    hookLeftMotor.set(VictorSPXControlMode.PercentOutput, power);
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
