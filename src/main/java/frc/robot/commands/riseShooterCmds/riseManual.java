// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.riseShooterCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RiseShooterSubsystem;

public class riseManual extends Command {
  /** Creates a new riseManual. */
    private final RiseShooterSubsystem riseShooterSubsytem;
  public riseManual(RiseShooterSubsystem riseShooterSubsystem) {
    this.riseShooterSubsytem = riseShooterSubsystem;
    addRequirements(this.riseShooterSubsytem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    riseShooterSubsytem.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    riseShooterSubsytem.manual();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    riseShooterSubsytem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
