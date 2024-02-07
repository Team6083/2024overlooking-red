// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<<< HEAD:src/main/java/frc/robot/commands/ShootPIDCmd.java
package frc.robot.commands;
========
package frc.robot.Command.ShooterCommand;
>>>>>>>> 43c1ff3 ([mod] check):src/main/java/frc/robot/Command/ShooterCommand/ShootPIDCmd.java

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootPIDCmd extends Command {
  /** Creates a new ShootPIDCmd. */
  private final ShooterSubsystem shooterSubsystem;
  public ShootPIDCmd(ShooterSubsystem shooterSubsystem) {
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(this.shooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.stopMotor();
    shooterSubsystem.setSetpoint(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setPIDRate();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
