// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoTimerCmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.IntakeCmd;
import frc.robot.commands.ApriltagCmd.FaceTag;
import frc.robot.commands.TrackingCmd.TrackingNoteClockwiseCmd;
import frc.robot.commands.riseShooterCmds.RiseShooterAutoControlCmd;
import frc.robot.commands.shooterCmds.ShootPIDCmd;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RiseShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drivebase;

public final class BlueMiddleCmdGroup {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(Drivebase drivebase, IntakeSubsystem intake,
      RiseShooterSubsystem riseShooter, double mainLeftTrigger, double mainRightTrigger,
      ShooterSubsystem shooter) {
    return Commands.sequence(
      new FaceTag(drivebase).withTimeout(1),
      new RiseShooterAutoControlCmd(riseShooter, mainLeftTrigger, mainRightTrigger),
      new ShootPIDCmd(shooter),

      new GoForwardCmd(drivebase).withTimeout(1),
      new GoLeftCmd(drivebase).withTimeout(1),
      new TrackingNoteClockwiseCmd(drivebase),
      new IntakeCmd(intake),
      new FaceTag(drivebase).withTimeout(1),
      new RiseShooterAutoControlCmd(riseShooter, mainLeftTrigger, mainRightTrigger),
      new ShootPIDCmd(shooter),

      new GoLeftCmd(drivebase),
      new TrackingNoteClockwiseCmd(drivebase),
      new IntakeCmd(intake),
      new FaceTag(drivebase).withTimeout(1),
      new RiseShooterAutoControlCmd(riseShooter, mainLeftTrigger, mainRightTrigger),
      new ShootPIDCmd(shooter),

      new GoLeftCmd(drivebase),
      new TrackingNoteClockwiseCmd(drivebase),
      new IntakeCmd(intake),
      new FaceTag(drivebase).withTimeout(1),
      new RiseShooterAutoControlCmd(riseShooter, mainLeftTrigger, mainRightTrigger),
      new ShootPIDCmd(shooter),

      new GoForwardCmd(drivebase).withTimeout(2),
      new TrackingNoteClockwiseCmd(drivebase),
      new IntakeCmd(intake),
      new GoBackCmd(drivebase).withTimeout(1),
      new FaceTag(drivebase).withTimeout(1),
      new RiseShooterAutoControlCmd(riseShooter, mainLeftTrigger, mainRightTrigger),
      new ShootPIDCmd(shooter),

      new GoForwardCmd(drivebase).withTimeout(1),
      new TrackingNoteClockwiseCmd(drivebase),
      new IntakeCmd(intake),
      new GoBackCmd(drivebase).withTimeout(1),
      new FaceTag(drivebase).withTimeout(1),
      new RiseShooterAutoControlCmd(riseShooter, mainLeftTrigger, mainRightTrigger),
      new ShootPIDCmd(shooter),

      new StopCmd(drivebase));
  }

  private BlueMiddleCmdGroup() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
