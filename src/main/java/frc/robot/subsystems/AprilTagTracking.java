package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AprilTagTracking extends SubsystemBase {
    public static NetworkTable table;
    public static NetworkTableEntry tx;// = table.getEntry("tx");// table.getEntry("tx");
    public static NetworkTableEntry ty;// = table.getEntry("ty");
    public static NetworkTableEntry ta;// = table.getEntry("ta");
    public static NetworkTableEntry tv;
    public static NetworkTableEntry tid;
    public static NetworkTableEntry tl;

    public static NetworkTableEntry BT;
    public static NetworkTableEntry CR;
    public static NetworkTableEntry TR;
    public static NetworkTableEntry CT;

    public static double v;
    public static double a;
    public static double x;
    public static double y;
    public static double area;
    public static double ID;
    public static double latency;

    public static double[] bt; // botpose_targetspace
    public static double[] cr;// camerapose_robotspace
    public static double[] tr; // targetpose_robotpose;
    public static double[] ct; // camerapose_targetspace

    public static double MyDistance;

    public final static double limelightLensHeightInches = 0;
    public final static double limelightMountAngleDegrees = 0;
    public static double targetOffsetAngle_Vertical;
    public static double angleToGoalDegrees;
    public static double angleToGoalRadians;
    public static double goalHeightInches;

    public static void init() {
        table = NetworkTableInstance.getDefault().getTable("limelight");

    }

    public static double getMyDistance() {
        // readValue();
        double target_height = getBT()[1]; // botpose in targetspace y
        double x_dis = getBT()[0];
        double z_dis = getBT()[2];
        double hori_dis = Math.pow(Math.pow(x_dis, 2) + Math.pow(z_dis, 2), 1.0 / 2);
        MyDistance = Math.pow(Math.pow(target_height, 2) + Math.pow(hori_dis, 2), 1.0
                / 2);

        return MyDistance;
    }

    public static double getTx() {
        init();
        x = table.getEntry("tx").getDouble(0);
        return x;
    }

    public static double getTy() {
        init();
        y = table.getEntry("ty").getDouble(0);
        return y;
    }

    public static double getTa() {
        init();
        a = table.getEntry("ta").getDouble(0);
        return a;
    }

    public static double getTv() {
        init();
        v = table.getEntry("tv").getDouble(0);
        return v;
    }

    public static double getTID() {
        init();
        ID = table.getEntry("tid").getDouble(0);
        return ID;
    }

    public static double getTl() {
        init();
        latency = table.getEntry("tl").getDouble(0);
        return latency;
    }

    public static double[] getBT() {
        init();
        bt = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        return bt;
    }

    public static void putDashboard() {
        // SmartDashboard.putNumber("hasTarget", getTv());
        SmartDashboard.putNumber("LimelightX", getTx());
        SmartDashboard.putNumber("LimelightY", getTy());
        // SmartDashboard.putNumber("LimelightArea", getTa());
        SmartDashboard.putNumber("LimelightID", getTID());
        SmartDashboard.putNumber("latency", getTl());

        // botpose in targetspace
        SmartDashboard.putNumber("bt_x", getBT()[0]);
        SmartDashboard.putNumber("bt_y", getBT()[1]);
        SmartDashboard.putNumber("bt_z", getBT()[2]);

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
