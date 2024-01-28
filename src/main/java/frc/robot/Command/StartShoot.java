// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.Shooter;

public class StartShoot extends Command {
  /** Creates a new StartShoot. */
  private final Shooter shooter;
  public StartShoot(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setShooterCondition();
  }
}
