// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.hookCmds;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HookSubsystem;

public class LinePIDCmd extends Command {
  /** Creates a new HookPIDCmd. */
  private final HookSubsystem hookSubsystem;
  private double lineMudify;
  private boolean pov270value;
  private boolean pov90value;
  private int pov90Count;
  private int pov270Count;
  public LinePIDCmd(HookSubsystem hookSubsystem,boolean pov270value,boolean pov90value) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.hookSubsystem=hookSubsystem;
    this.pov270value=pov270value;
    this.pov90value=pov90value;
    addRequirements(hookSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hookSubsystem.stopLineMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    hookSubsystem.setLineSetpoint(hookSubsystem.getLineSetpoint());//邏輯有問題，如果你的setpoint永遠是你的setpoint，那不就不會收線嗎
    hookSubsystem.linePIDControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hookSubsystem.stopLineMotor();
    hookSubsystem.linePIDControl();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
