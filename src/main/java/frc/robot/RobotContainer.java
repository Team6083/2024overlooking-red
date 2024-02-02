// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Command.RiseStooterCmd;
import frc.robot.Command.StartIntakeCmd;
import frc.robot.Command.StartShootCmd;
import frc.robot.Subsystem.IntakeSubsystem;
import frc.robot.Subsystem.RiseShooterSubsytem;
import frc.robot.Subsystem.ShooterSubsystem;

public class RobotContainer {
  private final CommandXboxController main = new CommandXboxController(0);
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final RiseShooterSubsytem RiseMotor = new RiseShooterSubsytem();

  public RobotContainer() {
    // intake.setDefaultCommand(new StartIntakeCmd(intake));
    configureBindings();

  }

  private void configureBindings() {
    main.x().toggleOnTrue(new StartShootCmd(shooter));
    main.b().toggleOnTrue(new StartIntakeCmd(intake));
    main.pov(0).whileTrue(new RiseStooterCmd(RiseMotor));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
