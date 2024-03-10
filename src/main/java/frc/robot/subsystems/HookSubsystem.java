// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HookConstants;

public class HookSubsystem extends SubsystemBase {
  /** Creates a new HookSubsystem. */
  private final PIDController linePID;
  private final PIDController hookLeftPID;
  private final PIDController hookRightPID;
  private final CANSparkMax lineMotor;
  public final VictorSPX hookLeftMotor;
  public final VictorSPX hookRightMotor;
  private final RelativeEncoder lineEncoder;
  private final DutyCycleEncoder hookEncoder;
  private final PowerDistributionSubsystem powerDistributionSubsystem;
  private double linePositionOffset = 0.0;
  private double hookLeftPositionOffset = 0.0;
  private double hookRightPositionOffset = 0.0;

  public HookSubsystem(PowerDistributionSubsystem powerDistributionSubsystem) {
    lineMotor = new CANSparkMax(HookConstants.kHookLineChannel, MotorType.kBrushless);
    hookLeftMotor = new VictorSPX(HookConstants.kHookLeftMotorChannel);
    hookRightMotor = new VictorSPX(HookConstants.kHookRightMotorChannel);
    linePID = new PIDController(HookConstants.kP, HookConstants.kI, HookConstants.kD);
    hookLeftPID = new PIDController(HookConstants.kP, HookConstants.kI, HookConstants.kD);
    hookRightPID = new PIDController(HookConstants.kP, HookConstants.kI, HookConstants.kD);
    lineEncoder = lineMotor.getEncoder();
    hookEncoder = new DutyCycleEncoder(HookConstants.kHookLeftEncoderChannel);
    lineEncoder.setPositionConversionFactor(HookConstants.kHookPositionConversionfactor);
    lineMotor.setInverted(HookConstants.kLineMotorInverted);
    hookLeftMotor.setInverted(HookConstants.kHookMotorLeftInverted);
    hookRightMotor.setInverted(HookConstants.kHookMotorRightInverted);
    this.powerDistributionSubsystem = powerDistributionSubsystem;
  }

  public Command hookDownLeftManualCmd() {
    Command cmd = this.runEnd(
        () -> this.manualControlLeftHookMotor(-HookConstants.kManualControlLeftHookMotorPower),
        this::stopHookLeftMotor);
    cmd.setName("hookDownLeftManualCmd");
    return cmd;
  }

  public Command hookDownRightManualCmd() {
    Command cmd = this.startEnd(
        () -> this.manualControlRightHookMotor(-HookConstants.kManualControlRightHookMotorPower),
        this::stopHookRightMotor);
    cmd.setName("hookDownRightManualCmd");
    return cmd;
  }

  public Command hookUpLeftManualCmd() {
    Command cmd = this.startEnd(
        () -> this.manualControlLeftHookMotor(-HookConstants.kManualControlRightHookMotorPower),
        this::stopHookLeftMotor);
    cmd.setName("hookUpLeftManualCmd");
    return cmd;
  }

  public Command hookupRightManualCmd() {
    Command cmd = this.startEnd(
        () -> this.manualControlRightHookMotor(-HookConstants.kManualControlRightHookMotorPower),
        this::stopHookRightMotor);
    cmd.setName("hookupRightManualCmd");
    return cmd;
  }

  public void manualControlLine(double hookControlSpeed) {
    setLineMotor(hookControlSpeed);
    linePID.setSetpoint(getLinePosition());
  }

  public void manualControlLeftHookMotor(double speed) {
    setLeftHookMotorPower(HookConstants.kHookMotorLeftVoltage);
    hookLeftPID.setSetpoint(getLeftMotorPosition());
  }

  public void manualControlRightHookMotor(double speed) {
    setRightHookMotorPower(HookConstants.kHookMotorRightVoltage);
    hookRightPID.setSetpoint(getRightMotorPosition());
  }

  public double getLineSetpoint() {
    return linePID.getSetpoint();
  }

  public double getLeftHookSetpoint() {
    return hookLeftPID.getSetpoint();
  }

  public double getRightHookSetpoint() {
    return hookRightPID.getSetpoint();
  }

  public void setLineSetpoint(double setpoint) {
    final double currentSetpoint = setpoint;

    if (isExceedPhysicalLine(currentSetpoint) != 0) {
      linePID.setSetpoint(
          (isExceedPhysicalLine(currentSetpoint)) >= 1 ? HookConstants.kLinePositionMax
              : HookConstants.kLinePositionMin);
      return;
    }

    linePID.setSetpoint(currentSetpoint);

  }

  public void setLeftHookMotorSetpoint(double leftMotorSetpoint) {
    final double currentSetpoint = leftMotorSetpoint;
    if (isExceedPhysicalLine(currentSetpoint) != 0) {
      return;
    }
    hookLeftPID.setSetpoint(currentSetpoint);
  }

  public void setRightHookMotorSetpoint(double RightMotorSetpoint) {
    final double currentSetpoint = RightMotorSetpoint;
    if (isExceedPhysicalLine(currentSetpoint) != 0) {
      return;
    }
    hookRightPID.setSetpoint(currentSetpoint);
  }

  public void linePIDControl() {
    double linePower = linePID.calculate(lineEncoder.getPosition());
    if (Math.abs(linePower) > HookConstants.kLinePower) {
      linePower = HookConstants.kLinePower * (linePower > 0 ? 1 : -1);
    }
    lineMotor.setVoltage(linePower);
    // SmartDashboard.putNumber("linepower", linePower);

  }

  public void hookLeftMotorPIDControl() {
    double hookMotorLeftVoltage = hookLeftPID.calculate(lineEncoder.getPosition(), getLeftHookSetpoint());
    if (Math.abs(hookMotorLeftVoltage) > HookConstants.kHookMotorLeftVoltage) {
      hookMotorLeftVoltage = HookConstants.kHookMotorLeftVoltage * (hookMotorLeftVoltage > 0 ? 1 : -1);
    }
    setLeftHookMotorVoltage(hookMotorLeftVoltage);
    // SmartDashboard.putNumber("hookmotor1power", hookMotorLeftVoltage);

  }

  public void hookRightMotorPIDControl() {
    double hookMotorRightVoltage = hookLeftPID.calculate(lineEncoder.getPosition(), getLeftHookSetpoint());
    if (Math.abs(hookMotorRightVoltage) > HookConstants.kHookMotorRightVoltage) {
      hookMotorRightVoltage = HookConstants.kHookMotorRightVoltage * (hookMotorRightVoltage > 0 ? 1 : -1);
    }
    setRightHookMotorVoltage(hookMotorRightVoltage);
    // SmartDashboard.putNumber("hookmotor2power", hookMotorRightVoltage);
  }

  public double getLinePosition() {
    // SmartDashboard.putNumber("LinePosition", lineEncoder.getPosition());
    return (lineEncoder.getPosition()) + linePositionOffset;
  }

  public double getLeftMotorPosition() {
    // SmartDashboard.putNumber("LeftPosition", hookEncoder.get());
    return (hookEncoder.get()) + hookLeftPositionOffset;
  }

  public double getRightMotorPosition() {
    // SmartDashboard.putNumber("RightPosition", hookEncoder.get());
    return (hookEncoder.get()) + hookRightPositionOffset;
  }

  public double getHookLeftMotorBusVoltage() {
    return hookLeftMotor.getBusVoltage();
  }

  public double getHookRightMotorBusVoltage() {
    return hookRightMotor.getBusVoltage();
  }

  public void stopLineMotor() {
    lineMotor.set(0.0);
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

  public void setRightHookMotorVoltage(double voltage) {
    setRightHookMotorPower(voltage / getHookRightMotorBusVoltage());
  }

  public void setLeftHookMotorPower(double power) {
    if (powerDistributionSubsystem.isHookLeftOverCurrent()) {
      stopHookLeftMotor();
      return;
    }
    hookLeftMotor.set(VictorSPXControlMode.PercentOutput, HookConstants.kHookMotorLeftVoltage);
  }

  public void setRightHookMotorPower(double power) {
    if (powerDistributionSubsystem.isHookRightOverCurrent()) {
      stopHookRightMotor();
      return;
    }
    hookRightMotor.set(VictorSPXControlMode.PercentOutput, HookConstants.kHookMotorRightVoltage);
  }

  public void setLineMotor(double voltage) {
    if (powerDistributionSubsystem.isLineMoterOverCurrent()) {
      stopLineMotor();
      return;
    }
    lineMotor.setVoltage(voltage);
  }

  public void resetEncoder() {
    lineEncoder.setPosition(0);
    hookEncoder.reset();
  }

  private int isExceedPhysicalLine(double position) {
    return (position < HookConstants.kLinePositionMin ? -1 : (position > HookConstants.kLinePositionMax) ? 1 : 0);

  }

  public double getAbsolutePosition() {
    return hookEncoder.getAbsolutePosition() - hookEncoder.getPositionOffset();
  }

  public double getDistance() {
    return hookEncoder.getDistance();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putData("LINEPID", linePID);
    // SmartDashboard.putData("left hook motor", hookLeftMotorPID);
    // SmartDashboard.putData("Right hook PID", hookRightMotorPID);
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("hookSubsystem");
    builder.addDoubleProperty("lineVoltage", () -> lineMotor.get() * lineMotor.getBusVoltage(), null);
    builder.addDoubleProperty("hookLeftMotorVoltage", hookLeftMotor::getMotorOutputVoltage, null);
    builder.addDoubleProperty("hookRightMotorVoltage", hookRightMotor::getMotorOutputVoltage, null);
    builder.addDoubleProperty("linePosition", lineEncoder::getPosition, null);
    builder.addDoubleProperty("hookAbsolutePosition", hookEncoder::getAbsolutePosition, null);
    linePID.initSendable(builder);
    hookLeftPID.initSendable(builder);
    hookRightPID.initSendable(builder);
  }
}
