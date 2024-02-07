// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

<<<<<<<< HEAD:src/main/java/frc/robot/commands/RiseShooterCmd.java
package frc.robot.commands;
========
package frc.robot.Command.RiseShooterCommand;
>>>>>>>> 43c1ff3 ([mod] check):src/main/java/frc/robot/Command/RiseShooterCommand/RiseShooterCmd.java

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.RiseShooterConstants;
import frc.robot.subsystems.RiseShooterSubsystem;

public class RiseShooterCmd extends Command {
  /** Creates a new RiseStooterCmd. */
  private final RiseShooterSubsystem riseShooterSubsytem;
  private double leftTriggerValue;
  private double rightTriggerValue;

  public RiseShooterCmd(RiseShooterSubsystem riseShooterSubsytem, double mainLeftTrigger, double mainRightTrigger) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.riseShooterSubsytem = riseShooterSubsytem;
    this.leftTriggerValue = mainLeftTrigger;
    this.rightTriggerValue = mainRightTrigger;
    addRequirements(this.riseShooterSubsytem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    riseShooterSubsytem.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double risePower = (leftTriggerValue - rightTriggerValue) * RiseShooterConstants.kriseTriggerValue;
    riseShooterSubsytem.manualControl(risePower);
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
