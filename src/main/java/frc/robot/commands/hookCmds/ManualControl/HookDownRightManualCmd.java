// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hookCmds.ManualControl;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HookConstants;
import frc.robot.subsystems.HookSubsystem;

public class HookDownRightManualCmd extends Command {
  /** Creates a new HookDownRightManualCmd. */
  private final HookSubsystem hookSubsystem;

  public HookDownRightManualCmd(HookSubsystem hookSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hookSubsystem = hookSubsystem;
    addRequirements(this.hookSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hookSubsystem.stopHookRightMotor();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hookSubsystem.manualControlRightHookMotor(-HookConstants.kmanualControlRightHookMotorPower);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hookSubsystem.stopHookRightMotor();
}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
