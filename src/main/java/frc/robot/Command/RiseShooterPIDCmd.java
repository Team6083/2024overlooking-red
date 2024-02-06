// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.RiseShooterSubsystem;

public class RiseShooterPIDCmd extends Command {
  private final RiseShooterSubsystem riseShooterSubsystem;
  private double leftTriggerValue;
  private double rightTriggerValue;
  private double armAngleModify;
  /** Creates a new RiseShooterPIDCmd. */
  public RiseShooterPIDCmd(RiseShooterSubsystem riseShooterSubsystem, double mainLeftTrigger, double mainRightTrigger) {
    this.riseShooterSubsystem = riseShooterSubsystem;
    this.leftTriggerValue = mainLeftTrigger;
    this.rightTriggerValue = mainRightTrigger;
    addRequirements(this.riseShooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    riseShooterSubsystem.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armAngleModify = (leftTriggerValue - rightTriggerValue) * -0.7;
    riseShooterSubsystem.setSetpoint(riseShooterSubsystem.getSetpoint() + armAngleModify);
    riseShooterSubsystem.pidControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    riseShooterSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
