package frc.robot.lib.Controller;

import frc.robot.lib.interFace.MotorControl;

public class CANSparkMaxController extends com.revrobotics.CANSparkMax implements MotorControl {

    public CANSparkMaxController(int deviceId, MotorType type) {
        super(deviceId, type);
        //TODO Auto-generated constructor stub
    }

    @Override
    public void setPower(double power) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPower'");
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
