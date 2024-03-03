// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.riseShooterCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RiseShooterSubsystem;

public class RiseShooterManualCmd extends Command {
  /** Creates a new RiseStooterCmd. */
  private final RiseShooterSubsystem riseShooterSubsystem;
  private double leftTriggerValue;
  private double rightTriggerValue;

  public RiseShooterManualCmd(RiseShooterSubsystem riseShooterSubsytem, double leftTriggerValue,
      double rightTriggerValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.riseShooterSubsystem = riseShooterSubsytem;
    this.rightTriggerValue = rightTriggerValue;
    this.leftTriggerValue = leftTriggerValue;
    addRequirements(this.riseShooterSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double risePower = (leftTriggerValue - rightTriggerValue) * 0.7;
    riseShooterSubsystem.manualControl(risePower);
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
