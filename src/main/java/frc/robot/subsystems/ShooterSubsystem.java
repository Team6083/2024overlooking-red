// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ApriltagTracking.TagTrackingLimelight;
import frc.robot.subsystems.TransportSubsystem;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new Shooter. */
  private final VictorSPX upMotor;
  private final VictorSPX downMotor;
  private final Encoder upEncoder;
  private final Encoder downEncoder;
  private final PIDController ratePidController;
  private final SimpleMotorFeedforward upMotorFeedForwardControl;
  private final SimpleMotorFeedforward downMotorFeedForwardControl;
  private final PowerDistributionSubsystem powerDistributionSubsystem;
  private final TransportSubsystem transportSubsystem;
  private final RiseShooterSubsystem riseShooterSubsystem;
  private final TagTrackingLimelight aprilTagTracking;

  public ShooterSubsystem(PowerDistributionSubsystem powerDistribution) {
    upMotor = new VictorSPX(ShooterConstants.kUpMotorChannel);
    downMotor = new VictorSPX(ShooterConstants.kDownMotorChannel);
    upEncoder = new Encoder(ShooterConstants.kUpEncoderChannelA, ShooterConstants.kUpEncoderChannelB);
    downEncoder = new Encoder(ShooterConstants.kDownEncoderChannelA,
        ShooterConstants.kDownEncoderChannelB);

    upEncoder.setReverseDirection(ShooterConstants.kUpEncoderInverted);
    downEncoder.setReverseDirection(ShooterConstants.kDownEncoderInverted);

    ratePidController = new PIDController(ShooterConstants.kP, ShooterConstants.kI, ShooterConstants.kD);

    upMotor.setInverted(ShooterConstants.kUpMotorInverted);
    downMotor.setInverted(ShooterConstants.kDownMotorInverted);

    upMotorFeedForwardControl = new SimpleMotorFeedforward(ShooterConstants.kUpMotorS, ShooterConstants.kUpMotorV,
        ShooterConstants.kUpMotorA);
    downMotorFeedForwardControl = new SimpleMotorFeedforward(ShooterConstants.kDownMotorS, ShooterConstants.kDownMotorV,
        ShooterConstants.kDownMotorA);

    resetEncoder();
    transportSubsystem = new TransportSubsystem(powerDistribution);
    aprilTagTracking = new TagTrackingLimelight();
    riseShooterSubsystem = new RiseShooterSubsystem(powerDistribution, aprilTagTracking);

    this.powerDistributionSubsystem = powerDistribution;
  }

  public void resetEncoder() {
    upEncoder.reset();
    downEncoder.reset();
  }

  public void setRateControl() {
    double rate = ShooterConstants.kShooterRate;
    final double upMotorVoltage = upMotorFeedForwardControl.calculate(rate)
        + ratePidController.calculate(getUpEncoderRate(), rate);
    final double downMotorVoltage = downMotorFeedForwardControl.calculate(rate)
        + ratePidController.calculate(getDownEncoderRate(), rate);
    setUpMotorVoltage(upMotorVoltage);
    setDownMotorVoltage(downMotorVoltage);
    SmartDashboard.putNumber("upMotorVoltage", upMotorVoltage);
    SmartDashboard.putNumber("downMotorVoltage", downMotorVoltage);
  }

  public void stopUpMotor() {
    upMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  public void stopDownMotor() {
    downMotor.set(VictorSPXControlMode.PercentOutput, 0.0);
  }

  public double getUpEncoderRate() {
    return upEncoder.getRate() / 2048.0;
  }

  public double getDownEncoderRate() {
    return downEncoder.getRate() / 2048.0;
  }

  public void setUpMotorVoltage(double voltage) {
    setUpMotor(voltage / getUpMotorBusVoltage());
  }

  public void setDownMotorVoltage(double voltage) {
    setDownMotor(voltage / getDownMotorBusVoltage());
  }

  public double getUpMotorBusVoltage() {
    return upMotor.getBusVoltage();
  }

  public double getDownMotorBusVoltage() {
    return downMotor.getBusVoltage();
  }

  public Boolean haveNoteAndSpeed() {
    if (transportSubsystem.isGetNote() && Math.abs(getUpEncoderRate() - ShooterConstants.kShooterRate) < 1
        && Math.abs(getDownEncoderRate() - ShooterConstants.kShooterRate) < 1) {
      return true;
    } else {
      return false;
    }
  }

  public void setUpMotor(double power) {
    if (powerDistributionSubsystem.isShooterUpOverCurrent()) {
      stopUpMotor();
      return;
    }
    upMotor.set(VictorSPXControlMode.PercentOutput, power);
  }

  public void setDownMotor(double power) {
    if (powerDistributionSubsystem.isShooterDownOverCurrent()) {
      stopDownMotor();
      return;
    }
    downMotor.set(VictorSPXControlMode.PercentOutput, power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("upRate", getUpEncoderRate());
    SmartDashboard.putNumber("downRate", getDownEncoderRate());
    SmartDashboard.putNumber("upPower", upMotor.getMotorOutputPercent());
    SmartDashboard.putNumber("downPower", downMotor.getMotorOutputPercent());

  }
}
