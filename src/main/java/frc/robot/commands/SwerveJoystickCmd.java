// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.subsystems.DrivebaseSubsystem;

public class SwerveJoystickCmd extends Command {
  /** Creates a new SwerveTest1ManualCmd. */
  private final DrivebaseSubsystem DrivebaseSubsystem;
  private final CommandXboxController main;
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter rotLimiter;
  private final double drivebaseMaxSpeed = DrivebaseConstants.kMaxSpeed;
  private double xSpeed, ySpeed, rotSpeed;

  public SwerveJoystickCmd(DrivebaseSubsystem drivebase, CommandXboxController main) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.DrivebaseSubsystem = drivebase;
    this.main = main;
    xLimiter = new SlewRateLimiter(DrivebaseConstants.xLimiterRateLimit);
    yLimiter = new SlewRateLimiter(DrivebaseConstants.yLimiterRateLimit);
    rotLimiter = new SlewRateLimiter(DrivebaseConstants.rotLimiterRateLimit);
    addRequirements(this.DrivebaseSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xSpeed = xLimiter.calculate(main.getLeftX()) * drivebaseMaxSpeed;
    ySpeed = yLimiter.calculate(main.getLeftY()) * drivebaseMaxSpeed;
    rotSpeed = rotLimiter.calculate(main.getRightX()) * drivebaseMaxSpeed;
    DrivebaseSubsystem.drive(-xSpeed, ySpeed, rotSpeed, !main.getHID().getAButton());
    // drivetain.testDrive(main.getLeftY(), main.getLeftX());
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
