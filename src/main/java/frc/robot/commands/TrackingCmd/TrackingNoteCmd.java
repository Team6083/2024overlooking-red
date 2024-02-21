// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrackingCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.NoteTracking.NoteTrackingPhotovision;
import frc.robot.subsystems.drive.Drivebase;

public class TrackingNoteCmd extends Command {
  /** Creates a new TrackingNoteCmd. */
  public final Drivebase drivebase;
  public TrackingNoteCmd(Drivebase drivebase) {
    this.drivebase = drivebase;
    addRequirements(drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivebase.facingNoteRot(0);
    drivebase.followingNoteSpeed();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}