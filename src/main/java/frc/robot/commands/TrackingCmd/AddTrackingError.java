// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TrackingCmd;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drivebase;

public class AddTrackingError extends Command {
  /** Creates a new AddTrackingError. */
  private final Drivebase drivebase;

  public AddTrackingError(Drivebase drivebase) {
    this.drivebase = drivebase;
    addRequirements(this.drivebase);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivebase.addTrackTargetError();
  }
}
