// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoTimerCmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ApriltagCmd.FollowNewCmd;
import frc.robot.commands.TrackingCmd.TrackingNoteCmd;
import frc.robot.commands.riseShooterCmds.RiseShooterAutoControlCmd;
import frc.robot.commands.shooterCmds.ShootPIDCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RiseShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drivebase;

public final class LeftNoSpeakerCmdGroup {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(Drivebase drivebase, IntakeSubsystem intake,
      RiseShooterSubsystem riseShooterSubsystem, double mainLeftTrigger, double mainRightTrigger,
      ShooterSubsystem shooterSubsystem) {
    return Commands.sequence(
        new FollowNewCmd(drivebase).withTimeout(1),
        new RiseShooterAutoControlCmd(riseShooterSubsystem, mainLeftTrigger, mainRightTrigger),
        new ShootPIDCmd(shooterSubsystem),
        new GoForwardCmd(drivebase).withTimeout(4),
        new TrackingNoteCmd(drivebase),
        new IntakeCmd(intake),
        new GoBackCmd(drivebase).withTimeout(2),
        new FollowNewCmd(drivebase).withTimeout(1),
        new RiseShooterAutoControlCmd(riseShooterSubsystem, mainLeftTrigger, mainRightTrigger),
        new ShootPIDCmd(shooterSubsystem),
        new StopCmd(drivebase));
  }

  private LeftNoSpeakerCmdGroup() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
