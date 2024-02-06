// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Command.RiseShooterCmd;
import frc.robot.Command.ShootManualCmd;
import frc.robot.Command.ShootPIDCmd;
import frc.robot.Command.SwerveJoystickCmd;
import frc.robot.Command.IntakeCmd;
import frc.robot.Constants.XboxControllerConstants;
import frc.robot.Subsystem.IntakeSubsystem;
import frc.robot.Subsystem.RiseShooterSubsytem;
import frc.robot.Subsystem.ShooterSubsystem;

public class RobotContainer {
  private final CommandXboxController main;
  private final ShooterSubsystem shooter;
  private final IntakeSubsystem intake;
  private final RiseShooterSubsytem riseMotor;
  

  double mainLeftTriggerValue;
  double mainRightTrigggerValue;

  public RobotContainer() {
    main = new CommandXboxController(XboxControllerConstants.kxbox);
    shooter = new ShooterSubsystem();
    intake = new IntakeSubsystem();
    riseMotor = new RiseShooterSubsytem();

    mainLeftTriggerValue = main.getLeftTriggerAxis();
    mainRightTrigggerValue = main.getRightTriggerAxis();
    configureBindings();

  }

  private void configureBindings() {
    main.y().and(main.y()).toggleOnTrue(new IntakeCmd(intake));
    main.a().and(main.a()).toggleOnTrue(new ShootManualCmd(shooter));
    main.a().and(main.a().negate()).toggleOnTrue(new ShootPIDCmd(shooter));
    // main.x().and(main.x()).toggleOnTrue(new RiseShooterCmd(riseMotor, mainLeftTriggerValue, mainRightTrigggerValue));
    riseMotor.setDefaultCommand(new RiseShooterCmd(riseMotor, mainLeftTriggerValue, mainRightTrigggerValue));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
