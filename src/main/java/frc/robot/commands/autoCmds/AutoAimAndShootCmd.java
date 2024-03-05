// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCmds;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ApriltagCmd.FaceTag;
import frc.robot.commands.riseShooterCmds.RiseShooterAutoControlCmd;
import frc.robot.commands.shooterCmds.ShootPIDCmd;
import frc.robot.subsystems.RiseShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.drive.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoAimAndShootCmd extends SequentialCommandGroup {
  /** Creates a new autoAimAndShootCmd. */
  public AutoAimAndShootCmd(Drivebase drivebase, RiseShooterSubsystem riseShooterSubsystem,
      ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new FaceTag(drivebase).withTimeout(0.5),
        new RiseShooterAutoControlCmd(riseShooterSubsystem).withTimeout(0.1),
        new ShootPIDCmd(shooterSubsystem));
  }
}
