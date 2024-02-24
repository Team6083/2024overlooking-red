// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PdConstants;

public class PowerDistributionSubsystem extends SubsystemBase {
  /** Creates a new PowerDistributionSubsystem. */
  private final PowerDistribution Pd;

  public PowerDistributionSubsystem() {
    Pd = new PowerDistribution();
  }

  public double intakeCurrent() {
    return Pd.getCurrent(PdConstants.kIntakeMotorCurrrentchannel);
  }

  public double hookLeftCurrent() {
    return Pd.getCurrent(PdConstants.kHookMotor1Currentchannel);
  }

  public double hookRightCurrent() {
    return Pd.getCurrent(PdConstants.kHookMotor2Currentchannel);
  }

  public double getDownShooterCurrent() {
    return Pd.getCurrent(PdConstants.kShooterDownMotorCurrentchannel);
  }

  public double getUpShooterCurrent() {
    return Pd.getCurrent(PdConstants.kShooterUpMotorCurrentchannel);
  }

  public double lineCurrent() {
    return Pd.getCurrent(PdConstants.klineCurrentchannel);
  }

  public double transportCurrent() {
    return Pd.getCurrent(PdConstants.kTransportCurrentchannel);
  }

  public boolean isIntakeOverCurrent() {
    return (Pd.getCurrent(PdConstants.kIntakeMotorCurrrentchannel) < PdConstants.kIntakeMotorMaxCurrent);
  }

  public boolean isHookLeftOverCurrent() {
    return (Pd.getCurrent(PdConstants.kHookMotor1Currentchannel) < PdConstants.kHookMotor1MaxCurrent);
  }

  public boolean isHookRightOverCurrent() {
    return (Pd.getCurrent(PdConstants.kHookMotor2Currentchannel) < PdConstants.kHookMotor2MaxCurrent);
  }

  public boolean isShooterDownOverCurrent() {
    return (Pd.getCurrent(PdConstants.kShooterDownMotorCurrentchannel) < PdConstants.kShooterDownMotorMaxCuurent);
  }

  public boolean isShooterUpOverCurrent() {
    return (Pd.getCurrent(PdConstants.kShooterUpMotorCurrentchannel) < PdConstants.kShooterUpMotorMaxCurrent);
  }

  public boolean isTransportOverCurrent() {
    return (Pd.getCurrent(PdConstants.kTransportCurrentchannel) < PdConstants.kTransportMaxCurrent);
  }

  public boolean isLineMoterOverCurrent() {
    return (Pd.getCurrent(PdConstants.klineCurrentchannel) < PdConstants.kLineMotorMaxCurrent);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("intakeCurrent", intakeCurrent());
    SmartDashboard.putNumber("hook1Current", hookLeftCurrent());
    SmartDashboard.putNumber("hook2Current", hookRightCurrent());
    SmartDashboard.putNumber("getDownShooterCurrent", getDownShooterCurrent());
    SmartDashboard.putNumber("getUpShooterCurrent", getUpShooterCurrent());
    SmartDashboard.putNumber("lineCurrent", lineCurrent());
    SmartDashboard.putNumber("kTransportCurrent", transportCurrent());

    SmartDashboard.putBoolean("intakePd", isIntakeOverCurrent());
    SmartDashboard.putBoolean("hook1Pd", isHookLeftOverCurrent());
    SmartDashboard.putBoolean("hook2Pd", isHookRightOverCurrent());
    SmartDashboard.putBoolean("downShooterPd", isShooterDownOverCurrent());
    SmartDashboard.putBoolean("upShooterPd", isShooterUpOverCurrent());
    SmartDashboard.putBoolean("transportPd", isTransportOverCurrent());
    SmartDashboard.putBoolean("lineMoterPd", isLineMoterOverCurrent());
  }
}
