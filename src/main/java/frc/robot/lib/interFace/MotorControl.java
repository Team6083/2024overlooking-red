package frc.robot.lib.interFace;

public interface MotorControl {
    public void setPower(double power);

    public void setVoltage(double voltage);

    public void setInverted(boolean isInverted);

    public double getOutputPecentage();

    public double getOutputVoltage();
}
