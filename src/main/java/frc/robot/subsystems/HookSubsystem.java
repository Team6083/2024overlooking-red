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
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HookConstants;

public class HookSubsystem extends SubsystemBase {
  /** Creates a new HookSubsystem. */
  private final PIDController linePID;
  private final PIDController hookLeftMotorPID;
  private final PIDController hookRightMotorPID;
  private final CANSparkMax line;
  public final VictorSPX hookLeftMotor;
  public final VictorSPX hookRightMotor;
  private final RelativeEncoder lineEncoder;
  private final Encoder hookRightEncoder;
  private final Encoder hookLeftEncoder;
  private final PowerDistributionSubsystem powerDistribution;
  private double linePositionOffset = 0.0;
  private double leftPositionOffset = 0.0;
  private double rightPositionOffset =0.0;

  public HookSubsystem(PowerDistributionSubsystem powerDistribution) {
    line = new CANSparkMax(HookConstants.kHookLineChannel, MotorType.kBrushless);
    hookLeftMotor = new VictorSPX(HookConstants.kHookLeftMotorCnannel);
    hookRightMotor = new VictorSPX(HookConstants.kHookRightMotorCnannel);
    linePID = new PIDController(HookConstants.kP, HookConstants.kI, HookConstants.kD);
    hookLeftMotorPID = new PIDController(HookConstants.kP, HookConstants.kI, HookConstants.kD);
    hookRightMotorPID = new PIDController(HookConstants.kP, HookConstants.kI, HookConstants.kD);
    lineEncoder = line.getEncoder();
    hookLeftEncoder = new Encoder(HookConstants.kHookLeftEncoderChannelA, HookConstants.kHookLeftEncoderChannelB);
    hookRightEncoder = new Encoder(HookConstants.kHookRightEncoderChannelA, HookConstants.kHookRightEncoderChannelB);
    lineEncoder.setPositionConversionFactor(HookConstants.kHookPositionConversionfactor);
    line.setInverted(HookConstants.kHookMotorLeftInverted);
    this.powerDistribution = powerDistribution;
  }

  public void manualControlLine(double hookControlSpeed) {
    setLineMotor(hookControlSpeed);
    linePID.setSetpoint(getLinePosition());
  }

  public void manualControlLeftHookMotor(double speed) {
    setLeftHookMotorPower(HookConstants.kHookMotorLeftVoltage);
    hookLeftMotorPID.setSetpoint(getLeftMotorPosition());
  }

  public void manualControlRightHookMotor(double speed) {
    setRightHookMotorPower(HookConstants.kHookMotorRightVoltage);
    hookRightMotorPID.setSetpoint(getRightMotorPosition());
  }

  public double getLineSetpoint() {
    return linePID.getSetpoint();
  }

  public double getLeftHookMotorSetpoint() {
    return hookLeftMotorPID.getSetpoint();
  }

  public double getRightHookMotorSetpoint() {
    return hookRightMotorPID.getSetpoint();
  }

  public void setLineSetpoint(double setSetpoint) {
    final double currentSetpoint = setSetpoint;

    if (isPhyLineExceed(currentSetpoint) != 0) {
      linePID.setSetpoint(
          (isPhyLineExceed(currentSetpoint)) >= 1 ? HookConstants.kLinePositionMax : HookConstants.kLinePositionMin);
      return;
    }

    linePID.setSetpoint(currentSetpoint);

  }

  public void setLeftHookMotorSetpoint(double leftMotorSetpoint) {
    final double currentSetpoint = leftMotorSetpoint;
    if (isPhyLineExceed(currentSetpoint) != 0) {
      return;
    }
    hookLeftMotorPID.setSetpoint(currentSetpoint);
  }

  public void setRightMotorSetpoint(double RightMotorSetpoint) {
    final double currentSetpoint = RightMotorSetpoint;
    if (isPhyLineExceed(currentSetpoint) != 0) {
      return;
    }
    hookRightMotorPID.setSetpoint(currentSetpoint);
  }

  public void linePIDControl() {
    double linePower = linePID.calculate(lineEncoder.getPosition());
    if (Math.abs(linePower) > HookConstants.kLinePower) {
      linePower = HookConstants.kLinePower * (linePower > 0 ? 1 : -1);
    }
    line.setVoltage(linePower);
    SmartDashboard.putNumber("linepower", linePower);

  }

  public void hookLeftMotorPIDControl() {
    double hookMotorLeftVoltage = hookLeftMotorPID.calculate(lineEncoder.getPosition(), getLeftHookMotorSetpoint());
    if (Math.abs(hookMotorLeftVoltage) > HookConstants.kHookMotorLeftVoltage) {
      hookMotorLeftVoltage = HookConstants.kHookMotorLeftVoltage * (hookMotorLeftVoltage > 0 ? 1 : -1);
    }
    setLeftHookMotorVoltage(hookMotorLeftVoltage);
    SmartDashboard.putNumber("hookmotor1power", hookMotorLeftVoltage);

  }

  public void hookRightMotorPIDControl() {
    double hookMotorRightVoltage = hookLeftMotorPID.calculate(lineEncoder.getPosition(), getLeftHookMotorSetpoint());
    if (Math.abs(hookMotorRightVoltage) > HookConstants.kHookMotorRightVoltage) {
      hookMotorRightVoltage = HookConstants.kHookMotorRightVoltage * (hookMotorRightVoltage > 0 ? 1 : -1);
    }
    setRightHookMotorVoltage(hookMotorRightVoltage);
    SmartDashboard.putNumber("hookmotor2power", hookMotorRightVoltage);
  }

  public double getLinePosition() {
    SmartDashboard.putNumber("LinePosition", lineEncoder.getPosition());
    return (lineEncoder.getPosition()) + linePositionOffset;
  }

  public double getLeftMotorPosition() {
    SmartDashboard.putNumber("LeftPosition", hookLeftEncoder.get());
    return (hookLeftEncoder.get()) + leftPositionOffset;
  }

  public double getRightMotorPosition() {
    SmartDashboard.putNumber("RightPosition", hookRightEncoder.get());
    return (hookRightEncoder.get()) + rightPositionOffset;
  }

  public double getHookLeftMotorBusVoltage() {
    return hookLeftMotor.getBusVoltage();
  }

  public double getHookRightMotorBusVoltage() {
    return hookRightMotor.getBusVoltage();
  }

  public void stopLineMotor() {
    line.set(0.0);
  }

  public void stopHookLeftMotor() {
    hookLeftMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  public void stopHookRightMotor() {
    hookRightMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  public void setLeftHookMotorVoltage(double voltage) {
    setLeftHookMotorPower(voltage / getHookLeftMotorBusVoltage());
  }

  public void setRightHookMotorVoltage(double voltage){
  setRightHookMotorPower(voltage/getHookRightMotorBusVoltage());
  }

  public void setLeftHookMotorPower(double power) {
    if (powerDistribution.isHookLeftOverCurrent()) {
      stopHookLeftMotor();
      return;
    }
    hookLeftMotor.set(VictorSPXControlMode.PercentOutput, HookConstants.kHookMotorLeftVoltage);
  }

  public void setRightHookMotorPower(double power) {
    if (powerDistribution.isHookRightOverCurrent()) {
      stopHookRightMotor();
      return;
    }
    hookRightMotor.set(VictorSPXControlMode.PercentOutput, HookConstants.kHookMotorRightVoltage);
  }

  public void setLineMotor(double voltage) {
    if (powerDistribution.isLineMoterOverCurrent()) {
      stopLineMotor();
      return;
    }
    line.setVoltage(voltage);
  }

  public void resetEncoder() {
    lineEncoder.setPosition(0);
    hookLeftEncoder.reset();
    hookRightEncoder.reset();
  }

  private int isPhyLineExceed(double position) {
    return (position < HookConstants.kLinePositionMin ? -1 : (position > HookConstants.kLinePositionMax) ? 1 : 0);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putData("LINEPID", linePID);
    SmartDashboard.putData("left hook motor", hookLeftMotorPID);
    SmartDashboard.putData("Right hook PID", hookRightMotorPID);
  }
}
