// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Command.StartShoot;
import frc.robot.Subsystem.Shooter;

public class RobotContainer {
  private final CommandXboxController main  = new CommandXboxController(0);
  private final Shooter shooter = new Shooter();
  public RobotContainer() {
    main.x().onTrue(new StartShoot(shooter));
    configureBindings();
    
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
