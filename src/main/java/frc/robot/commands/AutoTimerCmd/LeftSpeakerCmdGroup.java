// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoTimerCmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TrackingCmd.TrackingNoteCmd;
import frc.robot.subsystems.drive.Drivebase;

public final class LeftSpeakerCmdGroup {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(Drivebase drivebase) {
    return Commands.sequence(
        new GoForwardCmd(drivebase).withTimeout(2),
        new TrackingNoteCmd(drivebase),
        new GoLeftCmd(drivebase).withTimeout(2),
        new StopCmd(drivebase));
  }

  private LeftSpeakerCmdGroup() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
