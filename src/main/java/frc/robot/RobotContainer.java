// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.AutoTimerCmd.BlueLeftNoSpeakerCmdGroup;
import frc.robot.commands.AutoTimerCmd.BlueLeftSpeakerCmdGroup;
import frc.robot.commands.AutoTimerCmd.BlueMiddleCmdGroup;
import frc.robot.commands.AutoTimerCmd.BlueRightCmdGroup;
import frc.robot.commands.AutoTimerCmd.RedLeftCmdGroup;
import frc.robot.commands.AutoTimerCmd.RedRightNoSpeakerCmdGroup;
import frc.robot.commands.AutoTimerCmd.RedRightSpeakerCmdGroup;
import frc.robot.commands.AutoTimerCmd.RedMiddleCmdGroup;
import frc.robot.commands.AutoTimerCmd.StopCmd;
import frc.robot.commands.TransportCmds.IntakeTransCmd;
import frc.robot.commands.TransportCmds.ReTransCmd;
import frc.robot.commands.TransportCmds.TransCmd;
import frc.robot.commands.riseShooterCmds.RiseShooterManualCmd;
import frc.robot.commands.shooterCmds.ShootManualCmd;
import frc.robot.commands.shooterCmds.ShootPIDCmd;
import frc.robot.commands.shooterCmds.ShooterTestCmd;
import frc.robot.commands.GyroresetCmd;
import frc.robot.commands.hookCmds.HookManualCmd;
import frc.robot.commands.hookCmds.LinePIDCmd;
import frc.robot.commands.IntakeCmd;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.subsystems.AprilTagTracking;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.RiseShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.drive.Drivebase;

public class RobotContainer {
  private final CommandXboxController mainController;
  private final ShooterSubsystem shooter;
  private final TransportSubsystem trans;
  // private final HookSubsystem hook;
  private final IntakeSubsystem intake;
  private final RiseShooterSubsystem riseShooter;
  private final Drivebase drivebase;
  private final HookSubsystem hook;

  private SendableChooser<Command> autoChooser;

  double mainLeftTrigger;
  double mainRightTrigger;

  public RobotContainer() {
    mainController = new CommandXboxController(XboxControllerConstants.kMainController);
    shooter = new ShooterSubsystem();
    trans = new TransportSubsystem();
    intake = new IntakeSubsystem();
    riseShooter = new RiseShooterSubsystem();
    drivebase = new Drivebase();
    hook = new HookSubsystem();
    AprilTagTracking.init();

    mainLeftTrigger = mainController.getLeftTriggerAxis();
    mainRightTrigger = mainController.getRightTriggerAxis();
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    
    autoChooser.addOption("BlueRight", BlueRightCmdGroup.exampleAuto(drivebase, intake, riseShooter, mainLeftTrigger, mainRightTrigger, shooter));
    autoChooser.addOption("BlueMiddle", BlueMiddleCmdGroup.exampleAuto(drivebase, intake, riseShooter, mainLeftTrigger, mainRightTrigger, shooter));
    autoChooser.addOption("BlueLeftSpeaker", BlueLeftSpeakerCmdGroup.exampleAuto(drivebase, intake, riseShooter, mainLeftTrigger, mainRightTrigger, shooter));
    autoChooser.addOption("BlueLeftNospeaker", BlueLeftNoSpeakerCmdGroup.exampleAuto(drivebase, intake, riseShooter, mainLeftTrigger, mainRightTrigger, shooter));
    
        
    autoChooser.addOption("RedLeft", RedLeftCmdGroup.exampleAuto(drivebase, intake, riseShooter, mainLeftTrigger, mainRightTrigger, shooter));
    autoChooser.addOption("RedMiddle", RedMiddleCmdGroup.exampleAuto(drivebase, intake, riseShooter, mainLeftTrigger, mainRightTrigger, shooter));
    autoChooser.addOption("RedRightSpeaker", RedRightSpeakerCmdGroup.exampleAuto(drivebase, intake, riseShooter, mainLeftTrigger, mainRightTrigger, shooter));
    autoChooser.addOption("RedRightNospeaker", RedRightNoSpeakerCmdGroup.exampleAuto(drivebase, intake, riseShooter, mainLeftTrigger, mainRightTrigger, shooter));
    
    SmartDashboard.putData("Auto Choice", autoChooser);

    NamedCommands.registerCommand("ThrowIntoSpeaker", new ShootPIDCmd(shooter));
    NamedCommands.registerCommand("TransToShooter", new IntakeTransCmd(trans));
    NamedCommands.registerCommand("TakeNote", new IntakeCmd(intake));

  }

  private void configureBindings() {
    // main.y().toggleOnTrue(new IntakeCmd(intake).alongWith(new
    // IntakeTransCmd(trans)));
    // main.x().and(main.a()).toggleOnTrue(new ShootManualCmd(shooter));
    // riseMotor.setDefaultCommand(new RiseShooterManualCmd(riseMotor,
    // mainLeftTriggerValue, mainRightTrigggerValue));
    // drivebase.setDefaultCommand(new SwerveJoystickCmd(drivebase, main));
    // main.b().onTrue(new GyroresetCmd(drivebase) );
    mainController.a().toggleOnTrue(new ShootPIDCmd(shooter));
    mainController.x().toggleOnTrue(new TransCmd(trans));
    mainController.back().toggleOnTrue(new ReTransCmd(trans));
    // main.y().whileTrue(new HookManualCmd(hook));
    mainController.pov(0).onTrue(new LinePIDCmd(hook));
    mainController.pov(180).onTrue(new LinePIDCmd(hook));

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
