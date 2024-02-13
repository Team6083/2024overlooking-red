package frc.robot.lib.Controller;

import frc.robot.lib.interFace.MotorControl;

public class VictorSPController extends edu.wpi.first.wpilibj.motorcontrol.VictorSP implements MotorControl {

    public VictorSPController(int channel) {
        super(channel);
        //TODO Auto-generated constructor stub
    }

     @Override
    public void setPower(double power) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPower'");
    }

    @Override
    public void setVoltage(double voltage) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltage'");
    }

    @Override
    public double getOutputPecentage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getOutputPecentage'");
    }

    @Override
    public double getOutputVoltage() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getOutputVoltage'");
    }
    
}
