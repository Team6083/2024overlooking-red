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
  private PowerDistribution Pd;

  

  public PowerDistributionSubsystem() {
    this.Pd = Pd;
  }

  public double intakeCurrent(){
     return Pd.getCurrent(PdConstants.kIntakeMotorCurrrentchannel);
  }

  public double hook1Current(){
   return Pd.getCurrent(PdConstants.kHookMotor1Currentchannel);
  }

   public double hook2Current(){
   return Pd.getCurrent(PdConstants.kHookMotor2Currentchannel);
  }

  public double getDownShooterCurrent() {
    return Pd.getCurrent(PdConstants.kShooterDownMotorCurrentchannel);
  }

  public double getUpShooterCurrent() {
    return Pd.getCurrent(PdConstants.kShooterUpMotorCurrentchannel);
  } 

  public double lineCurrent(){
    return Pd.getCurrent(PdConstants.klineCurrentchannel);
  }

  public double kTransportCurrent(){
    return Pd.getCurrent(PdConstants.kTransportCurrentchannel);
  }


  public boolean intakePowerDistribution(){
   return(Pd.getCurrent(PdConstants.kIntakeMotorCurrrentchannel) < PdConstants.kIntakeMotorPd);   
  }

  public boolean hook1PowerDistribution(){
   return(Pd.getCurrent(PdConstants.kHookMotor1Currentchannel) < PdConstants.kHookMotor1Pd);   
  }
  
   public boolean hook2PowerDistribution(){
    return(Pd.getCurrent(PdConstants.kHookMotor2Currentchannel)< PdConstants.kHookMotor2Pd);
  }

  public boolean shooterDownPowerDistribution(){
    return(Pd.getCurrent(PdConstants.kShooterDownMotorCurrentchannel)< PdConstants.kShooterDownMoterrPd);
  }

   public boolean shooterUpPowerDistribution(){
    return(Pd.getCurrent(PdConstants.kShooterUpMotorCurrentchannel)< PdConstants.kShooterUpMotorPd);
  }

   public boolean transportPowerDistribution(){
    return(Pd.getCurrent(PdConstants.kTransportCurrentchannel)< PdConstants.kTransportPd);
  }

   public boolean lineMoterPowerDistribution(){
    return(Pd.getCurrent(PdConstants.klineCurrentchannel)< PdConstants.klineMotorPd);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      SmartDashboard.putNumber("intakeCurrent",intakeCurrent() );
      SmartDashboard.putNumber("hook1Current", hook1Current());
      SmartDashboard.putNumber("hook2Current", hook2Current());
      SmartDashboard.putNumber("getDownShooterCurrent", getDownShooterCurrent());
      SmartDashboard.putNumber("getUpShooterCurrent", getUpShooterCurrent());
      SmartDashboard.putNumber("lineCurrent", lineCurrent());
      SmartDashboard.putNumber("kTransportCurrent", kTransportCurrent());

      SmartDashboard.putBoolean("intakePd",intakePowerDistribution());
      SmartDashboard.putBoolean("hook1Pd",hook1PowerDistribution());
      SmartDashboard.putBoolean("hook2Pd",hook2PowerDistribution());
      SmartDashboard.putBoolean("downShooterPd", shooterDownPowerDistribution() );
      SmartDashboard.putBoolean("upShooterPd", shooterUpPowerDistribution());
      SmartDashboard.putBoolean("transportPd",transportPowerDistribution());
      SmartDashboard.putBoolean("lineMoterPd",lineMoterPowerDistribution());
  }
}
