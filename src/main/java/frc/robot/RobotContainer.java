// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.riseShooterCmds.RiseShooterManualCmd;
import frc.robot.commands.shooterCmds.ShootManualCmd;
import frc.robot.commands.shooterCmds.ShootPIDCmd;
import frc.robot.commands.GyroresetCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RiseShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
  private final CommandXboxController main;
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;
  private final RiseShooterSubsystem riseMotor;
  private final DrivebaseSubsystem drivebase;

  double mainLeftTriggerValue;
  double mainRightTrigggerValue;

  public RobotContainer() {
    main = new CommandXboxController(XboxControllerConstants.kMain);
    shooter = new ShooterSubsystem();
    intake = new IntakeSubsystem();
    riseMotor = new RiseShooterSubsystem();
    drivebase = new DrivebaseSubsystem();

    mainLeftTriggerValue = main.getLeftTriggerAxis();
    mainRightTrigggerValue = main.getRightTriggerAxis();
    configureBindings();

  }

  private void configureBindings() {
    main.y().toggleOnTrue(new IntakeCmd(intake));
    main.x().and(main.a()).toggleOnTrue(new ShootManualCmd(shooter));
    main.x().and(main.a().negate()).toggleOnTrue(new ShootPIDCmd(shooter));
    riseMotor.setDefaultCommand(new RiseShooterManualCmd(riseMotor, mainLeftTriggerValue, mainRightTrigggerValue));
    drivebase.setDefaultCommand(new SwerveJoystickCmd(drivebase, main));
    main.b().onTrue(new GyroresetCmd(drivebase) );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
