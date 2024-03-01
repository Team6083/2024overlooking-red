// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hookCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HookSubsystem;

public class HookRightMotorPIDCmd extends Command {
  /** Creates a new HookRightMotorPIDCmd. */
  private final HookSubsystem hookSubsystem;
  private double rightPositionMudify;
  private double rightTriggerValue;
  private double rightBumperValue;

  public HookRightMotorPIDCmd(HookSubsystem hookSubsystem, double rightTriggerValue, double rightBumperValue) {
    // Use addRequirements() here to declare subsystem dependencies
    this.hookSubsystem = hookSubsystem;
    this.rightBumperValue = rightBumperValue;
    this.rightTriggerValue = rightTriggerValue;
    addRequirements(hookSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hookSubsystem.stopHookRightMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rightPositionMudify = rightTriggerValue - rightBumperValue;
    hookSubsystem.setRightMotorSetpoint(hookSubsystem.getRightHookMotorSetpoint() + rightPositionMudify);
    hookSubsystem.hookRightMotorPIDControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hookSubsystem.stopHookRightMotor();
    hookSubsystem.hookRightMotorPIDControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
