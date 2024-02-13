package frc.robot.lib.Controller;

import frc.robot.lib.interFace.MotorControl;

public class VictorSPXController extends com.ctre.phoenix.motorcontrol.can.VictorSPX implements MotorControl {

    public VictorSPXController(int deviceNumber) {
        super(deviceNumber);
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
