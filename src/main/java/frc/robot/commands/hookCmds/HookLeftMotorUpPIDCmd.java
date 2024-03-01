// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hookCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HookConstants;
import frc.robot.subsystems.HookSubsystem;

public class HookLeftMotorUpPIDCmd extends Command {
  /** Creates a new HookMotorCmd. */
  private final HookSubsystem hookSubsystem;

  public HookLeftMotorUpPIDCmd(HookSubsystem hookSubsrystem) {
    this.hookSubsystem = hookSubsrystem;
    addRequirements(this.hookSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hookSubsystem.hookLeftMotorPIDControl();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hookSubsystem.setLeftHookMotorSetpoint(hookSubsystem.getLeftHookMotorSetpoint() + HookConstants.kLeftMotorModify);
    hookSubsystem.hookLeftMotorPIDControl();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hookSubsystem.hookLeftMotorPIDControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
