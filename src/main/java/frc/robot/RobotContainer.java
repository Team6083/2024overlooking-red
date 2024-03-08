// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveControllerConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.drive.Drivebase;
import frc.robot.subsystems.noteTracking.NoteTrackingPhotovision;
import frc.robot.commands.controllerCmds.DrivebaseAccelerateCmd;
import frc.robot.commands.controllerCmds.DrivebaseDefaultSpeedCmd;
import frc.robot.subsystems.PowerDistributionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.apriltagTracking.TagTrackingLimelight;
import frc.robot.subsystems.apriltagTracking.TagTrackingPhotonvision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.commands.hookCmds.ManualControl.HookUpLeftManualCmd;
import frc.robot.commands.hookCmds.ManualControl.HookUpRightManualCmd;
import frc.robot.commands.hookCmds.ManualControl.LineUpManualCmd;
import frc.robot.commands.hookCmds.PIDControl.HookLeftMotorDownPIDCmd;
import frc.robot.commands.hookCmds.PIDControl.HookLeftMotorUpPIDCmd;
import frc.robot.commands.hookCmds.PIDControl.HookRightMotorDownPIDCmd;
import frc.robot.commands.hookCmds.PIDControl.HookRightMotorUpPIDCmd;
import frc.robot.commands.hookCmds.PIDControl.LineDownPIDCmd;
import frc.robot.commands.hookCmds.PIDControl.LineUpPIDCmd;
// import frc.robot.subsystems.AprilTagTracking;
import frc.robot.subsystems.HookSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PowerDistributionSubsystem;
import frc.robot.subsystems.RotateShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TransportSubsystem;
import frc.robot.subsystems.drive.Drivebase;

public class RobotContainer {
  private final CommandXboxController mainController;
  private final CommandGenericHID controlPanel;
  private final PowerDistributionSubsystem powerDistributionSubsystem;
  private final Drivebase drivebase;
  // private final DrivebaseAccelerateCmd drivebaseAccelerate;
  // private final DrivebaseDefaultSpeedCmd drivebaseDefaultSpeed;
  private final IntakeSubsystem intake;
  private final TransportSubsystem transport;
  private final ShooterSubsystem shooter;
  private final RotateShooterSubsystem riseShooter;
  private final HookSubsystem hook;
  private final TagTrackingLimelight aprilTagTracking;
  private final NoteTrackingPhotovision noteTracking;
  private final TagTrackingPhotonvision photonTracking;

  private SendableChooser<Command> autoChooser;
  private SendableChooser<String> initialChooser;

  public RobotContainer() {
    powerDistributionSubsystem = new PowerDistributionSubsystem();
    aprilTagTracking = new TagTrackingLimelight();
    noteTracking = new NoteTrackingPhotovision();
    photonTracking = new TagTrackingPhotonvision();

    mainController = new CommandXboxController(DriveControllerConstants.kMainController);
    controlPanel = new CommandGenericHID(DriveControllerConstants.kControlPanel);
    drivebase = new Drivebase(noteTracking, aprilTagTracking, photonTracking);
    // drivebaseAccelerate = new DrivebaseAccelerateCmd(drivebase);
    // drivebaseDefaultSpeed = new DrivebaseDefaultSpeedCmd(drivebase);
    intake = new IntakeSubsystem(powerDistributionSubsystem);
    transport = new TransportSubsystem(powerDistributionSubsystem);
    shooter = new ShooterSubsystem(powerDistributionSubsystem);
    riseShooter = new RotateShooterSubsystem(powerDistributionSubsystem, aprilTagTracking);
    hook = new HookSubsystem(powerDistributionSubsystem);

    // AprilTagTracking.init();
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    // autoChooser = new SendableChooser<Command>();

    autoChooser.setDefaultOption("Do Nothing", Commands.none());
    // autoChooser.addOption("Forward", Autos.goStraightFroward(drivebase));
    // autoChooser.addOption("TurnRight", Autos.turnRight(drivebase));
    // autoChooser.addOption("Combine",
    // Autos.goStraightFrowardAndTurnRight(drivebase));
    // autoChooser.addOption(("Choreo Forward"),
    // Autos.choreoGoStraightForward(drivebase));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    initialChooser = new SendableChooser<String>();
    initialChooser.setDefaultOption("none", null);
    initialChooser.addOption("left", "left");
    initialChooser.addOption("middle", "middle");
    initialChooser.addOption("right", "right");
    SmartDashboard.putString("auto", null);
    SmartDashboard.putData(initialChooser);

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
    //drivetrain
    //drivebase.setDefaultCommand(new SwerveJoystickCmd(drivebase, main));
    //mainController.pov(0).toggleOnTrue(DrivebaseAccelerateCmd(drivebase));
    //mainController.pov(180).toggleOnTrue(DrivebaseDefaultSpeedCmd(drivebase));

    //riseshooter
    riseShooter.addErrorCommand(controlPanel.getRawAxis(0));
    //hook
    mainController.pov(270).whileTrue(new LineDownPIDCmd(hook));
    mainController.pov(90).whileTrue(new LineUpPIDCmd(hook));
    mainController.leftTrigger().whileTrue(new HookLeftMotorDownPIDCmd(hook));
    mainController.leftBumper().whileTrue(new HookLeftMotorUpPIDCmd(hook));
    mainController.rightTrigger().whileTrue(new HookRightMotorDownPIDCmd(hook));
    mainController.rightBumper().whileTrue(new HookRightMotorUpPIDCmd(hook));
  
    // intake and transport
    // mainController.y().toggleOnTrue(new IntakeWithTransportCmd(transport,
    // intake)); // onTrue could be okay, too
    // mainController.x().whileTrue(new ReIntakeWithTransportCmd(transport,
    // intake));

    //shooter
    //mainController.b().toggleOnTrue(new ShootPIDCmd(shooter));
    //mainController.a().toggleOnTrue(new ShootTransportCmd(transport, shooter.getRate()));
    
    //semi-automatic


    //reset
    //mainController.back().onTrue(new GyroResetCmd(drivebase));
  }

  public Command getAutonomousCommand() {
    String autoNumber = SmartDashboard.getString("auto", null);
    String initial = initialChooser.getSelected();
    var alliance = DriverStation.getAlliance();
    if (initial == null && alliance.isPresent())
      return new InstantCommand();
    Boolean isRed = alliance.get() == DriverStation.Alliance.Red;
    if (isRed) {
      initial = (initial == "left" ? "right" : (initial == "right" ? "left" : "middle"));
    }
    // return Autos.auto(drivebase, riseShooter, shooter, transport, intake,
    // autoNumber, initial);
    return Autos.autoOptimize(drivebase, riseShooter, shooter, transport, intake, autoNumber, initial);
    // return Commands.none();
  }

}
