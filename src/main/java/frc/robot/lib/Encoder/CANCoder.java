package frc.robot.lib.Encoder;

import frc.robot.lib.interFace.Encoder;

public class CANCoder extends com.ctre.phoenix6.hardware.CANcoder implements Encoder {

    public CANCoder(int deviceId) {
        super(deviceId);
        //TODO Auto-generated constructor stub
    }

    @Override
    public void setDistancePerPulse(double distancePerPulse) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDistancePerPulse'");
    }

    @Override
    public void setInverted(boolean isInverted) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
    }

    @Override
    public double getRobotPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRobotPosition'");
    }

    @Override
    public double getRobotAbsolutePoision() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRobotAbsolutePoision'");
    }

    @Override
    public double getRate() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getRate'");
    }
    
}
