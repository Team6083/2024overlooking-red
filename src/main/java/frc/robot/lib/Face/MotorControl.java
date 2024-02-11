package frc.robot.lib.Face;

public interface MotorControl {
    public void setPower(double power);

    public void setVoltage(double voltage);

    public double getOutputPecentage();

    public double getOutputVoltage();
}
