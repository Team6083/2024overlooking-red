// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.rotateShooterCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RotateShooterSubsystem;

public class RotateShooterManualCmd extends Command {
  /** Creates a new RiseStooterCmd. */
  private final RotateShooterSubsystem rotateShooterSubsystem;
  private double leftTriggerValue;
  private double rightTriggerValue;

  public RotateShooterManualCmd(RotateShooterSubsystem rotateShooterSubsytem, double leftTriggerValue,
      double rightTriggerValue) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.rotateShooterSubsystem = rotateShooterSubsytem;
    this.rightTriggerValue = rightTriggerValue;
    this.leftTriggerValue = leftTriggerValue;
    addRequirements(this.rotateShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      rotateShooterSubsystem.pidControl();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rotatePower = (leftTriggerValue - rightTriggerValue) * 0.7;
    rotateShooterSubsystem.manualControl(rotatePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    rotateShooterSubsystem.pidControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
