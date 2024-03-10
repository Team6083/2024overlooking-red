// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterAndRotate;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.rotateShooterCmds.RotatePutDownCmd;
import frc.robot.commands.rotateShooterCmds.RotateAMPCmd;
import frc.robot.subsystems.RotateShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AMPMode extends SequentialCommandGroup {
  /** Creates a new AMPMode. */
  public AMPMode(RotateShooterSubsystem rotateShooterSubsystem, ShooterSubsystem shooterSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RotateAMPCmd(rotateShooterSubsystem), 
      shooterSubsystem.ampShootPIDCmd()
    );
  }
}
