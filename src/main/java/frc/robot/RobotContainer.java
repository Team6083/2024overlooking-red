// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.TransportCmds.IntakeTransCmd;
import frc.robot.commands.TransportCmds.ReTransCmd;
import frc.robot.commands.TransportCmds.TransCmd;
import frc.robot.commands.riseShooterCmds.RiseShooterManualCmd;
import frc.robot.commands.shooterCmds.ShootManualCmd;
import frc.robot.commands.shooterCmds.ShootPIDCmd;
import frc.robot.commands.shooterCmds.ShooterTestCmd;
import frc.robot.commands.GyroresetCmd;
import frc.robot.commands.hookCmds.HookManualCmd;
import frc.robot.commands.hookCmds.HookManualPIDCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RiseShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.drive.DrivebaseSubsystem;

public class RobotContainer {
  private final CommandXboxController main;
  private final ShooterSubsystem shooter;
  private final TransportSubsystem trans;
  // private final HookSubsystem hook;
  private final IntakeSubsystem intake;
  // private final RiseShooterSubsystem riseMotor;
  // private final DrivebaseSubsystem drivebase;
  private final HookSubsystem hook;


  // double mainLeftTriggerValue;
  // double mainRightTrigggerValue;

  public RobotContainer() {
    main = new CommandXboxController(XboxControllerConstants.kMain);
    shooter = new ShooterSubsystem();
    trans = new TransportSubsystem();
    // hook = new HookSubsystem();
    intake = new IntakeSubsystem();
    // riseMotor = new RiseShooterSubsystem();
    // drivebase = new DrivebaseSubsystem();
    hook = new HookSubsystem();
    
    // mainLeftTriggerValue = main.getLeftTriggerAxis();
    // mainRightTrigggerValue = main.getRightTriggerAxis();
    configureBindings();

  }

  private void configureBindings() {
    // main.y().toggleOnTrue(new IntakeCmd(intake).alongWith(new IntakeTransCmd(trans)));
    // main.x().and(main.a()).toggleOnTrue(new ShootManualCmd(shooter));
    // riseMotor.setDefaultCommand(new RiseShooterManualCmd(riseMotor, mainLeftTriggerValue, mainRightTrigggerValue));
    // drivebase.setDefaultCommand(new SwerveJoystickCmd(drivebase, main));
    // main.b().onTrue(new GyroresetCmd(drivebase) );
    main.a().toggleOnTrue(new ShootPIDCmd(shooter));
    main.x().toggleOnTrue(new TransCmd(trans));
    main.back().toggleOnTrue(new ReTransCmd(trans));
    // main.y().whileTrue(new HookManualCmd(hook));
    main.pov(0).onTrue(new HookManualPIDCmd(hook));
    main.pov(180).onTrue(new HookManualPIDCmd(hook));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
