package frc.robot.subsystems.apriltagTracking;

import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.AprilTagConstants;

public class TagTrackingLimelight extends SubsystemBase {
    public NetworkTable table;

    public double v;
    public double a;
    public double x;
    public double y;
    public double area;
    public double ID;
    public double latency;

    public double[] bt; // botpose_targetspace
    public double[] ct; // camerapose_targetspace

    public double MyDistance;

    public final double limelightLensHeightInches = 0;
    public final double limelightMountAngleDegrees = 0;
    public double targetOffsetAngle_Vertical;
    public double angleToGoalDegrees;
    public double angleToGoalRadians;
    public double goalHeightInches;

    public TagTrackingLimelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public void setCamMode(int camMode) {
        table.getEntry("camMode").setNumber(camMode);
    }

    public void setLedMode(int ledMode) {
        table.getEntry("ledMode").setNumber(ledMode);
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

    public double[] getCT() {
        ct = table.getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
        return ct;
    }

    public double getHorizontalDis2() {
        double horDis = Math.sqrt((Math.pow(getBT()[0], 2) + Math.pow(getBT()[2], 2)));
        return horDis;
    }

    /**
     * Returns goal height in metres
     * 
     * @param offset tag to goal (metres)
     * @return goal height (metres)
     */
    public double getGoalHeight(double offset) {
        double goalHeight = Math.abs(getBT()[1]) + offset;
        return goalHeight;
    }

    /**
     * Returns bot to tag horizontal distance in metres
     * 
     * @return bot to tag horizontal distance (metres)
     */
    public double getHorizontalDis3() {
        double angle = getBT()[4]; // roll
        double angle_Radian = angle * (3.14159 / 180.0);
        double horDis = Math.abs(getBT()[2]) / Math.cos(angle_Radian);
        return horDis;
    }

    /**
     * Returns bot to speaker angle in degree
     * 
     * @return bot to speaker angle (degree)
     */
    public double getSpeakerDegree() {
        double degree = Math.atan(getGoalHeight(0.515) / getHorizontalDis3());
        return degree;
    }

    public double getTestSpeakerDegree() {
        double pitch = getTy();
        double yaw = getTx();
        double y = 0.615 * (1 / Math.tan(Math.toRadians(pitch + 10.0)));
        double x = y * Math.tan(Math.toRadians(yaw - 0.0))
                - 0;
        double distance = Math.sqrt(Math.pow(y, 2.0) + Math.pow(x, 2.0));
        double degree = Math.toDegrees(Math.atan((1.385 / distance)));
        return degree;
    }

    public double getTestTwoDegree(double nowDegree) {
        if (getTv() == 1) {
            double horDis = Math.abs(getBT()[2]) - 0.21;
            double degree = Math.toDegrees(Math.atan(1.6 / horDis));
            return degree;
        } else {
            return nowDegree;
        }
    }

    /**
     * Set priority tag iD
     * 
     * @param priorityID the priority tag ID (int)
     */
    public void setPriorityInViewTag(int priorityID) {
        table.getEntry("priorityid").setNumber(priorityID);
    }

    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    public void putDashboard() {
        // SmartDashboard.putNumber("hasTarget", getTv());
        SmartDashboard.putNumber("LimelightX", getTx());
        SmartDashboard.putNumber("LimelightY", getTy());
        // SmartDashboard.putNumber("LimelightArea", getTa());
        SmartDashboard.putNumber("LimelightID", getTID());
        SmartDashboard.putNumber("latency", getTl());

        SmartDashboard.putNumber("hor_Dis", getHorizontalDis3());
        SmartDashboard.putNumber("MyDistance", getMyDistance());

        SmartDashboard.putNumber("SpeakerDegree", getSpeakerDegree());
        SmartDashboard.putNumber("TestSpeakerDegree", getTestSpeakerDegree());
        SmartDashboard.putNumber("TestTwoDegree", getTestTwoDegree(30));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        putDashboard();
    }
}
