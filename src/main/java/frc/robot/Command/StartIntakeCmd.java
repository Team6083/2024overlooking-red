// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystem.IntakeSubsystem;

public class StartIntakeCmd extends Command {
  /** Creates a new StartIntake. */
  private final IntakeSubsystem intake;

  public StartIntakeCmd(IntakeSubsystem intake) {
    this.intake = intake;
    addRequirements(this.intake);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stopVoltage();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setVoltage();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopVoltage();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}