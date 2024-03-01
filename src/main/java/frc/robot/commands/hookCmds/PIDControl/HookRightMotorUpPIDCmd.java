// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hookCmds.PIDControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HookConstants;
import frc.robot.subsystems.HookSubsystem;

public class HookRightMotorUpPIDCmd extends Command {
  /** Creates a new HookRightMotorPIDCmd. */
  private final HookSubsystem hookSubsystem;

  public HookRightMotorUpPIDCmd(HookSubsystem hookSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies
    this.hookSubsystem = hookSubsystem;
    addRequirements(hookSubsystem);
  }

  // Called when the command is initially scheduled.

  @Override
  public void initialize() {
    hookSubsystem.hookRightMotorPIDControl();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hookSubsystem.hookRightMotorPIDControl();
    hookSubsystem.setRightMotorSetpoint(hookSubsystem.getRightHookMotorSetpoint() + HookConstants.kRightMotorModify);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hookSubsystem.hookRightMotorPIDControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
