// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.drive.Drivebase;

public class SwerveJoystickCmd extends Command {
  /** Creates a new SwerveTest1ManualCmd. */
  private final Drivebase drivebase;
  private final CommandXboxController main;
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter rotLimiter;
  private final double drivebaseMaxSpeed = DrivebaseConstants.kMaxSpeed;
  private double ySpeed, xSpeed, rotSpeed;

  public SwerveJoystickCmd(Drivebase drivebase, CommandXboxController main) {
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
    ySpeed = yLimiter.calculate(main.getLeftY()) * drivebaseMaxSpeed;// forward
    xSpeed = xLimiter.calculate(main.getLeftX()) * drivebaseMaxSpeed;// side
    rotSpeed = rotLimiter.calculate(main.getRightX()) * drivebaseMaxSpeed;
    drivebase.drive(-ySpeed, xSpeed, rotSpeed, true);
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
