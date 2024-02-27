// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PdConstants;

public class PowerDistributionSubsystem {
  private static PowerDistribution Pd;

  public static void init(){
    Pd = new PowerDistribution();
  }

  public static double intakeCurrent() {
    SmartDashboard.putNumber("intakeCurrent", intakeCurrent());
    return Pd.getCurrent(PdConstants.kIntakeMotorCurrrentchannel);
  }

  public static double hookLeftCurrent() {
    SmartDashboard.putNumber("hook1Current", hookLeftCurrent());
    return Pd.getCurrent(PdConstants.kHookMotor1Currentchannel);
  }

  public static double hookRightCurrent() {
     SmartDashboard.putNumber("hook2Current", hookRightCurrent());
    return Pd.getCurrent(PdConstants.kHookMotor2Currentchannel);
  }

  public static double getDownShooterCurrent() {
     SmartDashboard.putNumber("getDownShooterCurrent", getDownShooterCurrent());
    return Pd.getCurrent(PdConstants.kShooterDownMotorCurrentchannel);
  }

  public static double getUpShooterCurrent() {
    SmartDashboard.putNumber("getUpShooterCurrent", getUpShooterCurrent());
    return Pd.getCurrent(PdConstants.kShooterUpMotorCurrentchannel);
  }

  public static double lineCurrent() {
    SmartDashboard.putNumber("lineCurrent", lineCurrent());
    return Pd.getCurrent(PdConstants.klineCurrentchannel);
  }

  public static double transportCurrent() {
    SmartDashboard.putNumber("kTransportCurrent", transportCurrent());
    return Pd.getCurrent(PdConstants.kTransportCurrentchannel);
  }

  public static boolean isIntakeOverCurrent() {
    SmartDashboard.putBoolean("isIntakeOverCurren", isIntakeOverCurrent());
    return (intakeCurrent()> PdConstants.kIntakeMotorMaxCurrent);
  }

  public static boolean isHookLeftOverCurrent() {
     SmartDashboard.putBoolean("isHookLeftOverCurrent", isHookLeftOverCurrent());
    return (hookLeftCurrent() > PdConstants.kHookMotor1MaxCurrent);
  }

  public static boolean isHookRightOverCurrent() {
     SmartDashboard.putBoolean("isHookRightOverCurrent", isHookRightOverCurrent());
    return (hookRightCurrent() > PdConstants.kHookMotor2MaxCurrent);
  }

  public static boolean isShooterDownOverCurrent() {
      SmartDashboard.putBoolean("isShooterDownOverCurrent", isShooterDownOverCurrent());
    return (Pd.getCurrent(PdConstants.kShooterDownMotorCurrentchannel) > PdConstants.kShooterDownMotorMaxCuurent);
  }

  public static boolean isShooterUpOverCurrent() {
    SmartDashboard.putBoolean("isShooterUpOverCurren", isShooterUpOverCurrent());
    return (Pd.getCurrent(PdConstants.kShooterUpMotorCurrentchannel) > PdConstants.kShooterUpMotorMaxCurrent);
  }

  public static boolean isTransportOverCurrent() {
    SmartDashboard.putBoolean("isTransportOverCurrent", isTransportOverCurrent());
    return (Pd.getCurrent(PdConstants.kTransportCurrentchannel) > PdConstants.kTransportMaxCurrent);
    
  }

  public static boolean isLineMoterOverCurrent() {
     SmartDashboard.putBoolean("isLineMoterOverCurrent", isLineMoterOverCurrent());
    return (Pd.getCurrent(PdConstants.klineCurrentchannel) > PdConstants.kLineMotorMaxCurrent);
    
  }

  public static void periodic() {
    // This method will be called once per scheduler run

  }
}
