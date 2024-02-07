// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<<< HEAD:src/main/java/frc/robot/commands/ShootManualCmd.java
package frc.robot.commands;
========
package frc.robot.Command.ShooterCommand;
>>>>>>>> 43c1ff3 ([mod] check):src/main/java/frc/robot/Command/ShooterCommand/ShootManualCmd.java

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootManualCmd extends Command {
  /** Creates a new StartShoot. */
  private final ShooterSubsystem shooter;

  public ShootManualCmd(ShooterSubsystem shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
 
  @Override
  public void initialize(){
    shooter.stopMotor();
  }
 
  @Override
  public void execute() {
    shooter.setManualPercentage();;
  }

  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
