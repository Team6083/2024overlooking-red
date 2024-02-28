package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagTracking extends SubsystemBase {
    private final NetworkTable table;
    private NetworkTableEntry tx;// = table.getEntry("tx");// table.getEntry("tx");
    private NetworkTableEntry ty;// = table.getEntry("ty");
    private NetworkTableEntry ta;// = table.getEntry("ta");
    private NetworkTableEntry tv;
    private NetworkTableEntry tid;
    private NetworkTableEntry tl;

    private NetworkTableEntry BT;
    private NetworkTableEntry CR;
    private NetworkTableEntry TR;
    private NetworkTableEntry CT;

    private double v;
    private double a;
    private double x;
    private double y;
    private double area;
    private double ID;
    private double latency;

    private double[] bt; // botpose_targetspace
    private double[] cr;// camerapose_robotspace
    private double[] tr; // targetpose_robotpose;
    private double[] ct; // camerapose_targetspace

    private double MyDistance;

    private final double limelightLensHeightInches = 0;
    private final double limelightMountAngleDegrees = 0;
    private double targetOffsetAngle_Vertical;
    private double angleToGoalDegrees;
    private double angleToGoalRadians;
    private double goalHeightInches;

    public AprilTagTracking(){
        table = NetworkTableInstance.getDefault().getTable("limelight");

    }

    public double getMyDistance() {
        // readValue();
        double target_height = getBT()[1]; // botpose in targetspace y
        double x_dis = getBT()[0];
        double z_dis = getBT()[2];
        double hori_dis = Math.pow(Math.pow(x_dis, 2) + Math.pow(z_dis, 2), 1.0 / 2);
        MyDistance = Math.pow(Math.pow(target_height, 2) + Math.pow(hori_dis, 2), 1.0
                / 2);

        return MyDistance;
    }

    public double getTx() {
        x = table.getEntry("tx").getDouble(0);
        return x;
    }

    public double getTy() {
        y = table.getEntry("ty").getDouble(0);
        return y;
    }

    public double getTa() {
        a = table.getEntry("ta").getDouble(0);
        return a;
    }

    public double getTv() {
        v = table.getEntry("tv").getDouble(0);
        return v;
    }

    public double getTID() {
        ID = table.getEntry("tid").getDouble(0);
        return ID;
    }

    public double getTl() {
        latency = table.getEntry("tl").getDouble(0);
        return latency;
    }

    public double[] getBT() {
        bt = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        return bt;
    }

    public void putDashboard() {
        // SmartDashboard.putNumber("hasTarget", getTv());
        SmartDashboard.putNumber("LimelightX", getTx());
        SmartDashboard.putNumber("LimelightY", getTy());
        // SmartDashboard.putNumber("LimelightArea", getTa());
        SmartDashboard.putNumber("LimelightID", getTID());
        SmartDashboard.putNumber("latency", getTl());

        // botpose in targetspace
        SmartDashboard.putNumber("bt_x", getBT()[0]);//x distance
        SmartDashboard.putNumber("bt_y", getBT()[1]);//height
        SmartDashboard.putNumber("bt_z", getBT()[2]);//y distance

        // campose in targetspace
        // SmartDashboard.putNumber("ct_x", getCT()[0]);
        // SmartDashboard.putNumber("ct_y", getCT()[1]);
        // SmartDashboard.putNumber("ct_z", getCT()[2]);

        SmartDashboard.putNumber("MyDistance", getMyDistance());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        putDashboard();
    }
}
