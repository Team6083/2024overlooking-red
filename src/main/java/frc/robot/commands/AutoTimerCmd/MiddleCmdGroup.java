// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoTimerCmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.TrackingCmd.SwitchTrackConditionCmd;
import frc.robot.subsystems.drive.Drivebase;

public final class MiddleCmdGroup {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(Drivebase drivebase) {
    return Commands.sequence(
        new GoForwardCmd(drivebase).withTimeout(2),
        new SwitchTrackConditionCmd(drivebase),
        new StopCmd(drivebase));
  }

  private MiddleCmdGroup() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
