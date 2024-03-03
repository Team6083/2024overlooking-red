// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.ApriltagCmd.FaceTag;
import frc.robot.commands.autoTimerCmds.GoBackCmd;
import frc.robot.commands.autoTimerCmds.GoForwardCmd;
import frc.robot.commands.autoTimerCmds.GoLeftCmd;
import frc.robot.commands.autoTimerCmds.GoRightCmd;
import frc.robot.commands.autoTimerCmds.StopCmd;
import frc.robot.commands.intakeCmds.AutoIntakeCmd;
import frc.robot.commands.noteTrackingCmds.TrackingNoteClockwiseCmd;
import frc.robot.commands.noteTrackingCmds.TrackingNoteCounterclockwiseCmd;
import frc.robot.commands.riseShooterCmds.RiseShooterAutoControlCmd;
import frc.robot.commands.shooterCmds.ShootPIDCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RiseShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drivebase;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto() {
    return Commands.sequence();
  }

  // public static Command Example(Drivebase drivebase){
  // return drivebase.followPathCommand(AutoConstants.pathGoToSpeaker);
  // }

  // timer
  public static Command redLeftTrans(Drivebase drivebase, IntakeSubsystem intake, RiseShooterSubsystem riseShooter,
      ShooterSubsystem shooter) {
    return Commands.sequence(
        new ParallelCommandGroup(new FaceTag(drivebase),
            new RiseShooterAutoControlCmd(riseShooter)).withTimeout(1),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoLeftCmd(drivebase).withTimeout(1.5),
            new GoForwardCmd(drivebase).withTimeout(4),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new GoBackCmd(drivebase).withTimeout(2),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoRightCmd(drivebase).withTimeout(0.5),
            new GoForwardCmd(drivebase).withTimeout(1),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new GoLeftCmd(drivebase).withTimeout(0.5),
            new GoBackCmd(drivebase).withTimeout(1)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoRightCmd(drivebase).withTimeout(1),
            new GoForwardCmd(drivebase).withTimeout(1),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new GoLeftCmd(drivebase).withTimeout(1),
            new GoBackCmd(drivebase).withTimeout(1)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoRightCmd(drivebase).withTimeout(1.5),
            new GoForwardCmd(drivebase).withTimeout(1),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new GoLeftCmd(drivebase).withTimeout(1.5),
            new GoBackCmd(drivebase).withTimeout(1)),
        new ShootPIDCmd(shooter),

        new StopCmd(drivebase));
  }

  public static Command redMiddle(Drivebase drivebase, IntakeSubsystem intake, RiseShooterSubsystem riseShooter,
      ShooterSubsystem shooter) {
    return Commands.sequence(
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(0.5),
            new GoLeftCmd(drivebase).withTimeout(0.5),
            new TrackingNoteCounterclockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoRightCmd(drivebase).withTimeout(0.5),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoRightCmd(drivebase).withTimeout(0.5),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(4),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new GoBackCmd(drivebase).withTimeout(4),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new GoForwardCmd(drivebase).withTimeout(3),
        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(0.5),
            new GoLeftCmd(drivebase).withTimeout(0.5),
            new TrackingNoteCounterclockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new GoRightCmd(drivebase).withTimeout(0.5),
            new GoBackCmd(drivebase).withTimeout(4)),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new StopCmd(drivebase));
  }

  public static Command redRightTrans(Drivebase drivebase, IntakeSubsystem intake, RiseShooterSubsystem riseShooter,
      ShooterSubsystem shooter) {
    return Commands.sequence(
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(1),
            new TrackingNoteClockwiseCmd(drivebase)).withTimeout(1),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(4),
            new TrackingNoteClockwiseCmd(drivebase)).withTimeout(5),
        new AutoIntakeCmd(intake),
        new GoBackCmd(drivebase).withTimeout(2),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(2),
            new GoLeftCmd(drivebase).withTimeout(0.5),
            new TrackingNoteCounterclockwiseCmd(drivebase)).withTimeout(2.5),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new GoBackCmd(drivebase).withTimeout(2),
            new GoRightCmd(drivebase)).withTimeout(1),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(2),
            new GoLeftCmd(drivebase).withTimeout(1.5),
            new TrackingNoteCounterclockwiseCmd(drivebase)).withTimeout(3),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new GoBackCmd(drivebase).withTimeout(2),
            new GoRightCmd(drivebase).withTimeout(2.5)),
        new ShootPIDCmd(shooter),

        new StopCmd(drivebase));
  }

  public static Command redRight(Drivebase drivebase, IntakeSubsystem intake,
      RiseShooterSubsystem riseShooter, ShooterSubsystem shooter) {
    return Commands.sequence(
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(0.5),
            new GoLeftCmd(drivebase).withTimeout(1),
            new TrackingNoteCounterclockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoRightCmd(drivebase).withTimeout(0.5),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoRightCmd(drivebase).withTimeout(0.5),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(4),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new GoBackCmd(drivebase).withTimeout(4),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new GoForwardCmd(drivebase).withTimeout(3),
        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(0.5),
            new GoLeftCmd(drivebase).withTimeout(0.5),
            new TrackingNoteCounterclockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new GoRightCmd(drivebase).withTimeout(0.5),
            new GoBackCmd(drivebase).withTimeout(4)),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new StopCmd(drivebase));
  }

  public static Command blueRightTrans(Drivebase drivebase, IntakeSubsystem intake, RiseShooterSubsystem riseShooter,
      ShooterSubsystem shooter) {
    return Commands.sequence(
        new ParallelCommandGroup(new FaceTag(drivebase),
            new RiseShooterAutoControlCmd(riseShooter)).withTimeout(1),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoRightCmd(drivebase).withTimeout(1.5),
            new GoForwardCmd(drivebase).withTimeout(4),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new GoBackCmd(drivebase).withTimeout(2),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoLeftCmd(drivebase).withTimeout(0.5),
            new GoForwardCmd(drivebase).withTimeout(1),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new GoRightCmd(drivebase).withTimeout(0.5),
            new GoBackCmd(drivebase).withTimeout(1)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoLeftCmd(drivebase).withTimeout(1),
            new GoForwardCmd(drivebase).withTimeout(1),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new GoRightCmd(drivebase).withTimeout(1),
            new GoBackCmd(drivebase).withTimeout(1)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoLeftCmd(drivebase).withTimeout(1.5),
            new GoForwardCmd(drivebase).withTimeout(1),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new GoRightCmd(drivebase).withTimeout(1.5),
            new GoBackCmd(drivebase).withTimeout(1)),
        new ShootPIDCmd(shooter),

        new StopCmd(drivebase));
  }

  public static Command blueMiddle(Drivebase drivebase, IntakeSubsystem intake, RiseShooterSubsystem riseShooter,
      ShooterSubsystem shooter) {
    return Commands.sequence(
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(0.5),
            new GoRightCmd(drivebase).withTimeout(0.5),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoLeftCmd(drivebase).withTimeout(0.5),
            new TrackingNoteCounterclockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoLeftCmd(drivebase).withTimeout(0.5),
            new TrackingNoteCounterclockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(4),
            new TrackingNoteCounterclockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new GoBackCmd(drivebase).withTimeout(4),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new GoForwardCmd(drivebase).withTimeout(3),
        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(0.5),
            new GoRightCmd(drivebase).withTimeout(0.5),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new GoLeftCmd(drivebase).withTimeout(0.5),
            new GoBackCmd(drivebase).withTimeout(4)),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new StopCmd(drivebase));
  }

  public static Command blueLeftTrans(Drivebase drivebase, IntakeSubsystem intake, RiseShooterSubsystem riseShooter,
      ShooterSubsystem shooter) {
    return Commands.sequence(
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(1),
            new TrackingNoteCounterclockwiseCmd(drivebase)).withTimeout(1),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(4),
            new TrackingNoteCounterclockwiseCmd(drivebase)).withTimeout(5),
        new AutoIntakeCmd(intake),
        new GoBackCmd(drivebase).withTimeout(2),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(2),
            new GoRightCmd(drivebase).withTimeout(0.5),
            new TrackingNoteClockwiseCmd(drivebase)).withTimeout(2.5),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new GoBackCmd(drivebase).withTimeout(2),
            new GoLeftCmd(drivebase)).withTimeout(1),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(2),
            new GoRightCmd(drivebase).withTimeout(1.5),
            new TrackingNoteClockwiseCmd(drivebase)).withTimeout(3),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new GoBackCmd(drivebase).withTimeout(2),
            new GoLeftCmd(drivebase).withTimeout(2.5)),
        new ShootPIDCmd(shooter),

        new StopCmd(drivebase));
  }

  public static Command blueLeft(Drivebase drivebase, IntakeSubsystem intake,
      RiseShooterSubsystem riseShooter, ShooterSubsystem shooter) {
    return Commands.sequence(
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(0.5),
            new GoRightCmd(drivebase).withTimeout(1),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoLeftCmd(drivebase).withTimeout(0.5),
            new TrackingNoteCounterclockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoLeftCmd(drivebase).withTimeout(0.5),
            new TrackingNoteCounterclockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(4),
            new TrackingNoteCounterclockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new GoBackCmd(drivebase).withTimeout(4),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new GoForwardCmd(drivebase).withTimeout(3),
        new ParallelCommandGroup(new GoForwardCmd(drivebase).withTimeout(0.5),
            new GoRightCmd(drivebase).withTimeout(0.5),
            new TrackingNoteClockwiseCmd(drivebase)),
        new AutoIntakeCmd(intake),
        new ParallelCommandGroup(new GoLeftCmd(drivebase).withTimeout(0.5),
            new GoBackCmd(drivebase).withTimeout(4)),
        new ParallelCommandGroup(new FaceTag(drivebase).withTimeout(1),
            new RiseShooterAutoControlCmd(riseShooter)),
        new ShootPIDCmd(shooter),

        new StopCmd(drivebase));
  }

  // pathplanner
  public static Command RightTrans(Drivebase drivebase) {
    return drivebase.followAutoCommand(AutoConstants.rightTrans);
  }

  public static Command Middle(Drivebase drivebase) {
    return drivebase.followAutoCommand(AutoConstants.middle);
  }

  public static Command Left(Drivebase drivebase) {
    return drivebase.followAutoCommand(AutoConstants.left);
  }

  public static Command LeftTrans(Drivebase drivebase) {
    return drivebase.followAutoCommand(AutoConstants.leftTrans);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

}
