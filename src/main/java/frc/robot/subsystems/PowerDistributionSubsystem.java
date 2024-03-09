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
    SmartDashboard.putNumber("intakeCurrent", intakeCurrent());
    SmartDashboard.putNumber("hookLeftCurrent", hookLeftCurrent());
    SmartDashboard.putNumber("hookRightCurrent", hookRightCurrent());
    SmartDashboard.putNumber("upShooterCurrent", upShooterCurrent());
    SmartDashboard.putNumber("downShooterCurrent", downShooterCurrent());
    SmartDashboard.putNumber("lineCurrent", lineCurrent());
    SmartDashboard.putNumber("transportCurrent", transportCurrent());
    SmartDashboard.putNumber("rotateShooterCurrent", rotateShooterCurrent());
    SmartDashboard.putBoolean("isIntakeOverCurren", isIntakeOverCurrent());
    SmartDashboard.putBoolean("isHookLeftOverCurrent", isHookLeftOverCurrent());
    SmartDashboard.putBoolean("isHookRightOverCurrent", isHookRightOverCurrent());
    SmartDashboard.putBoolean("isShooterUpOverCurren", isShooterUpOverCurrent());
    SmartDashboard.putBoolean("isShooterDownOverCurrent", isShooterDownOverCurrent());
    SmartDashboard.putBoolean("isTransportOverCurrent", isTransportOverCurrent());
    SmartDashboard.putBoolean("isLineMoterOverCurrent", isLineMoterOverCurrent());
    SmartDashboard.putBoolean("isRotateShooterOverCurrent", isRotateShooterOverCurrent());
  }

  public double intakeCurrent() {
    SmartDashboard.putNumber("intakeCurrent", intakeCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kIntakeMotorCurrrentchannel);
  }

  public double hookLeftCurrent() {
    SmartDashboard.putNumber("hookLeftCurrent", hookLeftCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kHookMotor1Currentchannel);
  }

  public double hookRightCurrent() {
    SmartDashboard.putNumber("hookRightCurrent", hookRightCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kHookMotor2Currentchannel);
  }

  public double upShooterCurrent() {
    SmartDashboard.putNumber("upShooterCurrent", upShooterCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kShooterUpMotorCurrentchannel);
  }

  public double downShooterCurrent() {
    SmartDashboard.putNumber("downShooterCurrent", downShooterCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kShooterDownMotorCurrentchannel);
  }

  public double lineCurrent() {
    SmartDashboard.putNumber("lineCurrent", lineCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kLineCurrentchannel);
  }

  public double transportCurrent() {
    SmartDashboard.putNumber("transportCurrent", transportCurrent());
    return powerDistribution.getCurrent(PowerDistributionConstants.kTransportCurrentchannel);
  }

  public double rotateShooterCurrent() {
    SmartDashboard.putNumber("rotateShooterCurrent", rotateShooterCurrent());
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

  public boolean isShooterUpOverCurrent() {
    SmartDashboard.putBoolean("isShooterUpOverCurren", isShooterUpOverCurrent());
    return (upShooterCurrent() > PowerDistributionConstants.kShooterUpMotorMaxCurrent);
  }

  public boolean isShooterDownOverCurrent() {
    SmartDashboard.putBoolean("isShooterDownOverCurrent", isShooterDownOverCurrent());
    return (downShooterCurrent() > PowerDistributionConstants.kShooterDownMotorMaxCuurent);
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
    return (rotateShooterCurrent() > PowerDistributionConstants.kRotateShooterMaxCurrent);
  }

}
