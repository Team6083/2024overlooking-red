// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.transportCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransportSubsystem;

public class ReTransportIntakeCmd extends Command {
  /** Creates a new ReTrans. */
  private final TransportSubsystem transportSubsystem;

  public ReTransportIntakeCmd(TransportSubsystem transportSubsystem) {
    this.transportSubsystem = transportSubsystem;
    addRequirements(this.transportSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    transportSubsystem.stopMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    transportSubsystem.setReTransport();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transportSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}