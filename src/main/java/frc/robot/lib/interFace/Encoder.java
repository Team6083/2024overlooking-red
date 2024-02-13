package frc.robot.lib.interFace;

public interface Encoder {
    public void setDistancePerPulse(double distancePerPulse);

    public void setInverted(boolean isInverted);

    public double getRobotPosition();

    public double getRobotAbsolutePoision();

    public double getRate();

    
}
