package frc.robot.subsystems.ApriltagTracking;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.AprilTagConstants;

public class TagTrackingLimelight extends SubsystemBase {
    public NetworkTable table;
    public AprilTagFieldLayout m_layout;

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
        setCamMode(0);
        setLedMode(0);
        setPipeline(0);
        try {
            m_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException err) {
            throw new RuntimeException();
        }
    }

    /**
     * Set desired limelight operation mode. 0 is vision processor. 1 is Driver
     * Camera (Increases exposure, disables vision processing)
     * 
     * @param camMode set 0 plz
     */
    public void setCamMode(int camMode) {
        table.getEntry("camMode").setNumber(camMode);
    }

    /**
     * Set desired green light state. 0 is default. 1 is force off. 2 is force
     * blink. 3 is force on.
     * 
     * @param ledMode set 0 or 1 plz
     */
    public void setLedMode(int ledMode) {
        table.getEntry("ledMode").setNumber(ledMode);
    }

    /**
     * Set desired limelight pipeline.
     * 
     * @param pipeline in this game let's set 0
     */
    public void setPipeline(int pipeline) {
        table.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Returns bot to tag's direct distance.
     * 
     * @return distance (double)
     */
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

    /**
     * Returns the x offset between the tag and crosshair.
     * 
     * @return x offset
     */
    public double getTx() {
        x = table.getEntry("tx").getDouble(0);
        return x;
    }

    /**
     * Returns the y offset between the tag and crosshair.
     * 
     * @return y offset
     */
    public double getTy() {
        y = table.getEntry("ty").getDouble(0);
        return y;
    }

    /**
     * Returns 1 if a tag is detected. 0 if none.
     * 
     * @return 0 or 1
     */
    public double getTv() {
        v = table.getEntry("tv").getDouble(0);
        return v;
    }

    /**
     * Returns the fiducial tag's ID (double)
     * 
     * @return tag ID
     */
    public double getTID() {
        ID = table.getEntry("tid").getDouble(0);
        return ID;
    }

    /**
     * Returns the pipeline's total latency.
     * 
     * @return latency (double)
     */
    public double getTl() {
        latency = table.getEntry("tl").getDouble(0) + table.getEntry("cl").getDouble(0);
        return latency;
    }

    /**
     * Returns a double array of botpose in target space. The former 3 refers to
     * translation, while the latter 3 refers to rotation (in the sequence of roll,
     * pitch, yaw) In target space, (0,0,0) is the centre of the tag, x+ points to
     * the right side (when you're facing the tag), y+ points down, z+ points to
     * front.
     * 
     * @return x, y, z, roll, pitch, yaw
     */
    public double[] getBT() {
        bt = table.getEntry("botpose_targetspace").getDoubleArray(new double[6]);
        return bt;
    }

        /**
     * Returns a double array of campose in target space. The former 3 refers to
     * translation, while the latter 3 refers to rotation (in the sequence of roll,
     * pitch, yaw) In target space, (0,0,0) is the centre of the tag, x+ points to
     * the right side (when you're facing the tag), y+ points down, z+ points to
     * front.
     * 
     * @return x, y, z, roll, pitch, yaw
     */
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

    // public double getTestSpeakerDegree() {
    //     double pitch = getTy();
    //     double yaw = getTx();
    //     double y = 0.615 * (1 / Math.tan(Math.toRadians(pitch + 10.0)));
    //     double x = y * Math.tan(Math.toRadians(yaw - 0.0))
    //             - 0;
    //     double distance = Math.sqrt(Math.pow(y, 2.0) + Math.pow(x, 2.0));
    //     double degree = Math.toDegrees(Math.atan((1.385 / distance)));
    //     return degree;
    // }

    // public double getTestTwoDegree(double nowDegree) {
    //     if (getTv() == 1) {
    //         double horDis = Math.abs(getBT()[2]) - 0.21;
    //         double degree = Math.toDegrees(Math.atan(1.6 / horDis));
    //         return degree;
    //     } else {
    //         return nowDegree;
    //     }
    // }

    /**
     * Gets the tag's pose in 2 dimension
     * 
     * @return tagPose
     */
    public Pose2d getTagPose2d() {
        if (getTv() != 0) {
            Optional<Pose3d> tag_Pose3d = m_layout.getTagPose((int) getTID());
            Pose2d tagPose2d = tag_Pose3d.isPresent() ? tag_Pose3d.get().toPose2d() : new Pose2d();
            return tagPose2d;
        } else {
            return new Pose2d();
        }
    }

    /**
     * Gets the tag's pose in 3 dimension
     * 
     * @return tagPose
     */
    public Pose3d getTagPose3d() {
        if (getTv() != 0) {
            Optional<Pose3d> tag_Pose3d = m_layout.getTagPose((int) getTID());
            Pose3d tagPose = tag_Pose3d.isPresent() ? tag_Pose3d.get() : new Pose3d();
            return tagPose;
        } else {
            return new Pose3d();
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

    public void putDashboard() {
        // SmartDashboard.putNumber("hasTarget", getTv());
        SmartDashboard.putNumber("LimelightX", getTx());
        SmartDashboard.putNumber("LimelightY", getTy());
        // SmartDashboard.putNumber("LimelightArea", getTa());
        SmartDashboard.putNumber("LimelightID", getTID());
        SmartDashboard.putNumber("latency", getTl());

        SmartDashboard.putNumber("hor_Dis", getHorizontalDis3());
        SmartDashboard.putNumber("MyDistance", getMyDistance());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        putDashboard();
    }
}
