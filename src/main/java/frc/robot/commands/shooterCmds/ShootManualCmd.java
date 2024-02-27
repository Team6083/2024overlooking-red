// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooterCmds;

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
    shooter.stopUpMotor();
    shooter.stopDownMotor();
  
  }
 
  @Override
  public void execute() {
    shooter.setManual();
  }

  @Override
  public void end(boolean interrupted) {
     shooter.stopUpMotor();
    shooter.stopDownMotor();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
