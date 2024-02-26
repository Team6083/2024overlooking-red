package frc.robot.commands.TransportCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransportSubsystem;

public class IntakeTransCmd extends Command{
    
  /** Creates a new ReTrans. */
  private final TransportSubsystem transportSubsystem;

  public IntakeTransCmd(TransportSubsystem transportSubsystem) {
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
    transportSubsystem.intakeTrans();
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

