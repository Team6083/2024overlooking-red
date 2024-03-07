// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.controllerCmds;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.drive.Drivebase;

public class NoteFacingDrive extends Command {
  /** Creates a new NoteDrive. */
  private final Drivebase drivebase;
  private final CommandXboxController main;
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter rotLimiter;
  private final double drivebaseMaxSpeed = DrivebaseConstants.kMaxSpeed;
  private double xSpeed, ySpeed, rotSpeed, magnification;

  public NoteFacingDrive(Drivebase drivebase, CommandXboxController main) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivebase = drivebase;
    this.main = main;
    yLimiter = new SlewRateLimiter(DrivebaseConstants.kYLimiterRateLimit);
    xLimiter = new SlewRateLimiter(DrivebaseConstants.kXLimiterRateLimit);
    rotLimiter = new SlewRateLimiter(DrivebaseConstants.kRotLimiterRateLimit);
    addRequirements(this.drivebase);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    magnification = drivebase.getMagnification();
    xSpeed = yLimiter.calculate(main.getLeftY()) * drivebaseMaxSpeed * magnification; // forward
    ySpeed = xLimiter.calculate(main.getLeftX()) * drivebaseMaxSpeed * magnification; // side
    drivebase.drive(xSpeed, ySpeed, drivebase.facingNoteRot(0.0), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
