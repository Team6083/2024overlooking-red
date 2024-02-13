package frc.robot.lib.Encoder;

import edu.wpi.first.wpilibj.DigitalSource;
import frc.robot.lib.interFace.Encoder;

public class QuadratureEncoder extends edu.wpi.first.wpilibj.Encoder implements Encoder {

    public QuadratureEncoder(DigitalSource sourceA, DigitalSource sourceB) {
        super(sourceA, sourceB);
        //TODO Auto-generated constructor stub
    }

    @Override
    public void setInverted(boolean isInverted) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setInverted'");
    }

    @Override
    public double getRobotPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getPosition'");
    }

    @Override
    public double getRobotAbsolutePoision() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getAbsolutePoision'");
    }
    
}
