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
import frc.robot.commands.TeleopIntakeCmd;
import frc.robot.commands.autoTimerCmds.StopCmd;
import frc.robot.commands.riseShooterCmds.RiseShooterManualCmd;
import frc.robot.commands.riseShooterCmds.RiseShooterPIDCmd;
import frc.robot.commands.shooterCmds.ShootPIDCmd;
import frc.robot.commands.Autos;
import frc.robot.commands.hookCmds.HookManualCmd;
import frc.robot.commands.hookCmds.LinePIDCmd;
import frc.robot.commands.AutoIntakeCmd;
import frc.robot.Constants.XboxControllerConstants;
// import frc.robot.subsystems.AprilTagTracking;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PowerDistributionSubsystem;
import frc.robot.subsystems.RiseShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.ApriltagTracking.TagTrackingLimelight;
import frc.robot.subsystems.drive.Drivebase;

public class RobotContainer {
  private final CommandXboxController mainController;
  // private final ShooterSubsystem shooter;
  // private final TransportSubsystem trans;
  // private final HookSubsystem hook;
  // private final IntakeSubsystem intake;
  private final RiseShooterSubsystem riseShooter;
  // private final Drivebase drivebase;
  // private final HookSubsystem hook;
  private final PowerDistributionSubsystem powerDistribution;
  private final TagTrackingLimelight aprilTagTracking;

  private SendableChooser<Command> autoChooser;

  public RobotContainer() {
    powerDistribution = new PowerDistributionSubsystem();
    aprilTagTracking = new TagTrackingLimelight();

    mainController = new CommandXboxController(XboxControllerConstants.kMainController);
    // shooter = new ShooterSubsystem(powerDistribution);
    // trans = new TransportSubsystem(powerDistribution);
    // intake = new IntakeSubsystem(powerDistribution);
    riseShooter = new RiseShooterSubsystem(powerDistribution, aprilTagTracking);
    // drivebase = new Drivebase();
    // hook = new HookSubsystem(powerDistribution);
    // AprilTagTracking.init();
    configureBindings();

    // autoChooser = AutoBuilder.buildAutoChooser();

    // autoChooser.setDefaultOption("DoNothing", new StopCmd(drivebase));
    // autoChooser.addOption("Right", RightCmdGroup.exampleAuto(drivebase, intake,
    // riseShooter, mainLeftTrigger, mainRightTrigger, shooter));
    // autoChooser.addOption("Middle", MiddleCmdGroup.exampleAuto(drivebase, intake,
    // riseShooter, mainLeftTrigger, mainRightTrigger, shooter));
    // autoChooser.addOption("LeftSpeaker",
    // LeftSpeakerCmdGroup.exampleAuto(drivebase, intake, riseShooter,
    // mainLeftTrigger, mainRightTrigger, shooter));
    // autoChooser.addOption("LeftNospeaker",
    // LeftNoSpeakerCmdGroup.exampleAuto(drivebase, intake, riseShooter,
    // mainLeftTrigger, mainRightTrigger, shooter));
    // SmartDashboard.putData("Auto Choice", autoChooser);
    // NamedCommands.registerCommand("ThrowIntoSpeaker", new ShootPIDCmd(shooter));
    // NamedCommands.registerCommand("TransToShooter",new IntakeTransCmd(trans));
    // NamedCommands.registerCommand("TakeNote", new
    // IntakeCmd(intake).withTimeout(2));

  }

  private void configureBindings() {
    // mainController.y().toggleOnTrue(new TeleopIntakeCmd(intake,
    // trans.isGetNote()).alongWith(new
    // IntakeTransCmd(trans)));
    riseShooter.setDefaultCommand(
        new RiseShooterPIDCmd(riseShooter, mainController.getLeftTriggerAxis(), mainController.getRightTriggerAxis()));
    // mainController.a().toggleOnTrue(new RiseShooterManualCmd(riseShooter));
    // drivebase.setDefaultCommand(new SwerveJoystickCmd(drivebase, main));
    // main.b().onTrue(new GyroResetCmd(drivebase) );
    // mainController.a().toggleOnTrue(new ShootPIDCmd(shooter));
    // mainController.x().toggleOnTrue(new TransCmd(trans));
    // mainController.back().toggleOnTrue(new ReTransCmd(trans));
    // main.y().whileTrue(new HookManualCmd(hook));
    // mainController.pov(0).onTrue(new LinePIDCmd(hook));
    // mainController.pov(180).onTrue(new LinePIDCmd(hook));
    TagTrackingLimelight.init();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
