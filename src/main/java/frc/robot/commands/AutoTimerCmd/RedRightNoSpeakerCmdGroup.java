// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoTimerCmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ApriltagCmd.FaceTag;
import frc.robot.commands.TrackingCmd.TrackingNoteClockwiseCmd;
import frc.robot.commands.TrackingCmd.TrackingNoteCounterclockwiseCmd;
import frc.robot.commands.riseShooterCmds.RiseShooterAutoControlCmd;
import frc.robot.commands.shooterCmds.ShootPIDCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RiseShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drivebase;

public final class RedRightNoSpeakerCmdGroup {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(Drivebase drivebase, IntakeSubsystem intake,
      RiseShooterSubsystem riseShooter, double mainLeftTrigger, double mainRightTrigger,
      ShooterSubsystem shooter) {
    return Commands.sequence(
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(1),
            new TrackingNoteClockwiseCmd(drivebase)).withTimeout(1),
        new IntakeCmd(intake),
        new FaceTag(drivebase).withTimeout(1),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(4),
            new TrackingNoteClockwiseCmd(drivebase)).withTimeout(5),
        new IntakeCmd(intake),
        new GoBackCmd(drivebase).withTimeout(2),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(2),
            new GoLeftCmd(drivebase).withTimeout(0.5),
            new TrackingNoteCounterclockwiseCmd(drivebase)).withTimeout(2.5),
        new IntakeCmd(intake),
        new ParallelCommandGroup(new GoBackCmd(drivebase).withTimeout(2),
            new GoLeftCmd(drivebase)).withTimeout(1),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(2),
            new GoLeftCmd(drivebase).withTimeout(1.5),
            new TrackingNoteCounterclockwiseCmd(drivebase)).withTimeout(3),
        new IntakeCmd(intake),
        new ParallelCommandGroup(new GoBackCmd(drivebase).withTimeout(2),
            new GoLeftCmd(drivebase).withTimeout(2.5)),
        new ShootPIDCmd(shooter),

        new StopCmd(drivebase));
  }

  private RedRightNoSpeakerCmdGroup() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
