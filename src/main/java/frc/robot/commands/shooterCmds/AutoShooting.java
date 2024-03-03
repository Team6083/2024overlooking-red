// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RiseShooterSubsystem;

public class AutoShooting extends Command {
  /** Creates a new AutoShooting. */
  private final RiseShooterSubsystem riseShooterSubsystem;
  private final double leftTriggerValue;
  private final double rightTriggerValue;
  double armAngleModify;
  public AutoShooting(RiseShooterSubsystem riseShooterSubsystem, double mainLeftTrigger, double mainRightTrigger, boolean hasNoteAndSpeed) {
    this.riseShooterSubsystem = riseShooterSubsystem;
    this.leftTriggerValue = mainLeftTrigger;
    this.rightTriggerValue = mainRightTrigger;
    addRequirements(this.riseShooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    riseShooterSubsystem.pidControl();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armAngleModify = (leftTriggerValue - rightTriggerValue) * 0.7;
    riseShooterSubsystem.setSetpoint(riseShooterSubsystem.getSetpoint() + armAngleModify);
    riseShooterSubsystem.pidControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {   
    riseShooterSubsystem.pidControl();

  }
    
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
