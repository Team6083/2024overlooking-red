// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RiseShooterSubsystem;

public class AutoRiseShooterCmd extends Command {
  RiseShooterSubsystem riseShooterSubsystem;

  /** Creates a new AutoAimCmd. */
  public AutoRiseShooterCmd(RiseShooterSubsystem riseShooterSubsystem) {
    this.riseShooterSubsystem = riseShooterSubsystem;
    addRequirements(this.riseShooterSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    riseShooterSubsystem.setSetpoint(riseShooterSubsystem.getAprilTagDegree(riseShooterSubsystem.getSetpoint()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    riseShooterSubsystem.setSetpoint(riseShooterSubsystem.getAprilTagDegree(riseShooterSubsystem.getSetpoint()));
    riseShooterSubsystem.pidControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    riseShooterSubsystem.pidControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
