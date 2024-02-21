// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoTimerCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivebase;

public class GoForwardCmd extends Command {
  /** Creates a new GoForwardCmd. */
  public final Drivebase drivebase;

  public GoForwardCmd(Drivebase drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.drive(0.3, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}