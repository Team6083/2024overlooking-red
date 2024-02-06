// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Command;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RiseShooterConstants;
import frc.robot.Subsystem.RiseShooterSubsytem;

public class RiseStooterCmd extends Command {
  /** Creates a new RiseStooterCmd. */
  private final RiseShooterSubsytem riseShooterSubsytem;
  private final RiseShooterConstants riseShooterConstants;
  private double leftTriggerValue;
  private double rightTriggerValue;

  public RiseStooterCmd(RiseShooterSubsytem riseShooterSubsytem,double mainLeftTrigger,double mainrightTrigger) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.riseShooterSubsytem=riseShooterSubsytem;
    this.leftTriggerValue = mainLeftTrigger;
    this.rightTriggerValue = mainrightTrigger;
    addRequirements(this.riseMotor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    riseMotor.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double risePower = (leftTriggerValue - rightTriggerValue)*RiseShooterConstants.kriseTriggerValue;
    riseMotor.riseShooterControl(risePower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    riseMotor.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
