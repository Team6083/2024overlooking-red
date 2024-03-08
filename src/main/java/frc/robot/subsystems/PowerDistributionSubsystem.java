// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PowerDistributionConstants;

public class PowerDistributionSubsystem {
  private PowerDistribution powerDistribution;

  public PowerDistributionSubsystem() {
    powerDistribution = new PowerDistribution();
  }

  public double intakeCurrent() {
    SmartDashboard.putNumber("intakeCurrent", intakeCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kIntakeMotorCurrrentchannel);
  }

  public double hookLeftCurrent() {
    SmartDashboard.putNumber("hook1Current", hookLeftCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kHookMotor1Currentchannel);
  }

  public double hookRightCurrent() {
    SmartDashboard.putNumber("hook2Current", hookRightCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kHookMotor2Currentchannel);
  }

  public double getDownShooterCurrent() {
    SmartDashboard.putNumber("getDownShooterCurrent", getDownShooterCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kShooterDownMotorCurrentchannel);
  }

  public double getUpShooterCurrent() {
    SmartDashboard.putNumber("getUpShooterCurrent", getUpShooterCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kShooterUpMotorCurrentchannel);
  }

  public double lineCurrent() {
    SmartDashboard.putNumber("lineCurrent", lineCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kLineCurrentchannel);
  }

  public double transportCurrent() {
    SmartDashboard.putNumber("transportCurrent", transportCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kTransportCurrentchannel);
  }

  public double riseShooterCurrent() {
    SmartDashboard.putNumber("rotateShooterCurrent", riseShooterCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kRiseShooterCurrentchannel);
  }

  public boolean isIntakeOverCurrent() {
    SmartDashboard.putBoolean("isIntakeOverCurren", isIntakeOverCurrent());
    return (intakeCurrent() > PowerDistributionConstants.kIntakeMotorMaxCurrent);
  }

  public boolean isHookLeftOverCurrent() {
    SmartDashboard.putBoolean("isHookLeftOverCurrent", isHookLeftOverCurrent());
    return (hookLeftCurrent() > PowerDistributionConstants.kHookMotor1MaxCurrent);
  }

  public boolean isHookRightOverCurrent() {
    SmartDashboard.putBoolean("isHookRightOverCurrent", isHookRightOverCurrent());
    return (hookRightCurrent() > PowerDistributionConstants.kHookMotor2MaxCurrent);
  }

  public boolean isShooterDownOverCurrent() {
    SmartDashboard.putBoolean("isShooterDownOverCurrent", isShooterDownOverCurrent());
    return (getDownShooterCurrent() > PowerDistributionConstants.kShooterDownMotorMaxCuurent);
  }

  public boolean isShooterUpOverCurrent() {
    SmartDashboard.putBoolean("isShooterUpOverCurren", isShooterUpOverCurrent());
    return (getUpShooterCurrent() > PowerDistributionConstants.kShooterUpMotorMaxCurrent);
  }

  public boolean isTransportOverCurrent() {
    SmartDashboard.putBoolean("isTransportOverCurrent", isTransportOverCurrent());
    return (transportCurrent() > PowerDistributionConstants.kTransportMaxCurrent);

  }

  public boolean isLineMoterOverCurrent() {
    SmartDashboard.putBoolean("isLineMoterOverCurrent", isLineMoterOverCurrent());
    return (lineCurrent() > PowerDistributionConstants.kLineMotorMaxCurrent);

  }

  public boolean isRotateShooterOverCurrent() {
    SmartDashboard.putBoolean("isRotateShooterOverCurrent", isRotateShooterOverCurrent());
    return (riseShooterCurrent() > PowerDistributionConstants.kRotateShooterMaxCurrent);
  }

}
