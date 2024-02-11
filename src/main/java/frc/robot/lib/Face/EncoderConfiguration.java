package frc.robot.lib.Face;

public interface EncoderConfiguration {
    public void setDistancePerPulse(double distancePerPulse);

    public void setInverted(boolean isInverted);

    public double getPosition();

    public double getAbsolutePoision();

    public double getRate();

    
}
