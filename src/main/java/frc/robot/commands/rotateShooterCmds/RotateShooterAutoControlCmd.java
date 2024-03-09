// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.rotateShooterCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotateShooterSubsystem;

public class RotateShooterAutoControlCmd extends Command {
  private final RotateShooterSubsystem rotateShooterSubsystem;

  /** Creates a new RiseShooterPIDCmd. */
  public RotateShooterAutoControlCmd(RotateShooterSubsystem rotateShooterSubsystem) {
    this.rotateShooterSubsystem = rotateShooterSubsystem;
    addRequirements(this.rotateShooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotateShooterSubsystem.setSetpoint(rotateShooterSubsystem.getAimDegree(rotateShooterSubsystem.getSetpoint()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rotateShooterSubsystem.pidControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rotateShooterSubsystem.pidControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
