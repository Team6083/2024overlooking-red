// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PdConstants;

public class PowerDistributionSubsystem {
  private PowerDistribution Pd;

  public PowerDistributionSubsystem() {
    Pd = new PowerDistribution();
  }

  public double intakeCurrent() {
    SmartDashboard.putNumber("intakeCurrent", intakeCurrent());
    return Pd.getCurrent(PdConstants.kIntakeMotorCurrrentchannel);
  }

  public double hookLeftCurrent() {
    SmartDashboard.putNumber("hook1Current", hookLeftCurrent());
    return Pd.getCurrent(PdConstants.kHookMotor1Currentchannel);
  }

  public double hookRightCurrent() {
    SmartDashboard.putNumber("hook2Current", hookRightCurrent());
    return Pd.getCurrent(PdConstants.kHookMotor2Currentchannel);
  }

  public double getDownShooterCurrent() {
    SmartDashboard.putNumber("getDownShooterCurrent", getDownShooterCurrent());
    return Pd.getCurrent(PdConstants.kShooterDownMotorCurrentchannel);
  }

  public double getUpShooterCurrent() {
    SmartDashboard.putNumber("getUpShooterCurrent", getUpShooterCurrent());
    return Pd.getCurrent(PdConstants.kShooterUpMotorCurrentchannel);
  }

  public double lineCurrent() {
    SmartDashboard.putNumber("lineCurrent", lineCurrent());
    return Pd.getCurrent(PdConstants.kLineCurrentchannel);
  }

  public double transportCurrent() {
    SmartDashboard.putNumber("kTransportCurrent", transportCurrent());
    return Pd.getCurrent(PdConstants.kTransportCurrentchannel);
  }

  public double riseShooterCurrent() {
    SmartDashboard.putNumber("kriseShooterCurrent", riseShooterCurrent());
    return Pd.getCurrent(PdConstants.kRiseShooterCurrentchannel);
  }

  public boolean isIntakeOverCurrent() {
    SmartDashboard.putBoolean("isIntakeOverCurren", isIntakeOverCurrent());
    return (intakeCurrent() > PdConstants.kIntakeMotorMaxCurrent);
  }

  public boolean isHookLeftOverCurrent() {
    SmartDashboard.putBoolean("isHookLeftOverCurrent", isHookLeftOverCurrent());
    return (hookLeftCurrent() > PdConstants.kHookMotor1MaxCurrent);
  }

  public boolean isHookRightOverCurrent() {
    SmartDashboard.putBoolean("isHookRightOverCurrent", isHookRightOverCurrent());
    return (hookRightCurrent() > PdConstants.kHookMotor2MaxCurrent);
  }

  public boolean isShooterDownOverCurrent() {
    SmartDashboard.putBoolean("isShooterDownOverCurrent", isShooterDownOverCurrent());
    return (getDownShooterCurrent() > PdConstants.kShooterDownMotorMaxCuurent);
  }

  public boolean isShooterUpOverCurrent() {
    SmartDashboard.putBoolean("isShooterUpOverCurren", isShooterUpOverCurrent());
    return (getUpShooterCurrent() > PdConstants.kShooterUpMotorMaxCurrent);
  }

  public boolean isTransportOverCurrent() {
    SmartDashboard.putBoolean("isTransportOverCurrent", isTransportOverCurrent());
    return (transportCurrent() > PdConstants.kTransportMaxCurrent);

  }

  public boolean isLineMoterOverCurrent() {
    SmartDashboard.putBoolean("isLineMoterOverCurrent", isLineMoterOverCurrent());
    return (lineCurrent() > PdConstants.kLineMotorMaxCurrent);

  }

  public boolean isRiseShooterOverCurrent() {
    SmartDashboard.putBoolean("isRiseShooterOverCurrent", isRiseShooterOverCurrent());
    return (riseShooterCurrent() > PdConstants.kRiseShooterMaxCurrent);
  }

}
