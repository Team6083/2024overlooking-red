package frc.robot.subsystems.visionProcessing;

import java.io.IOException;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TagTracking {
    private final NetworkTable table;
    private final AprilTagFieldLayout m_layout;

    private double v;
    private double a;
    private double x;
    private double y;
    private double area;
    private double ID;
    private double latency;

    private double[] bt; // botpose_targetspace
    private double[] ct; // camerapose_targetspace

    private double distance;

    public TagTracking() {
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
    public double getDistance() {
        // readValue();
        double targetHeight = getBT()[1]; // botpose in targetspace y
        double xDis = getBT()[0];
        double zDis = getBT()[2];
        double horDis = Math.sqrt(Math.pow(xDis, 2) + Math.pow(zDis, 2));
        distance = Math.sqrt(Math.pow(targetHeight, 2) + Math.pow(horDis, 2));
        return distance;
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

    public double getHorizontalDistance() {
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
    public double getHorizontalDisByRoll() {
        double angleDegree = getBT()[4]; // roll
        double angleRadian = Math.toRadians(angleDegree);
        double horDis = Math.abs(getBT()[2]) / Math.cos(angleRadian);
        return horDis;
    }

    /**
     * Returns bot to speaker angle in degree
     * 
     * @return bot to speaker angle (degree)
     */
    public double getSpeakerDegree() {
        double degree = Math.toDegrees(Math.atan(getGoalHeight(0.515) / getHorizontalDisByRoll()));
        return degree;
    }

    /**
     * Not yet experimented. Return shooter to goal angle degree by calculating with
     * tx and ty.
     * 
     * @return shooter to goal angle (degree)
     */
    public double getSpeakerDegreeByCal() {
        double pitch = getTy();
        double yaw = getTx();
        double y = 0.615 * (1 / Math.tan(Math.toRadians(pitch + 10.0)));
        double x = y * Math.tan(Math.toRadians(yaw - 0.0))
                - 0;
        double distance = Math.sqrt(Math.pow(y, 2.0) + Math.pow(x, 2.0));
        double degree = Math.toDegrees(Math.atan((1.385 / distance)));
        return degree;
    }

    /**
     * Not yet experimented. Rerurn shooter to goal angle degree by botpose
     * targetspace when tag detected
     * 
     * @param nowDegree shooter degree calculated by encoder
     * @return shooter to goal angle (degree)
     */
    public double getSpeakerDegreeByBT(double nowDegree) {
        if (getTv() == 1) {
            double horDis = Math.abs(getBT()[2]) - 0.21;
            double degree = Math.toDegrees(Math.atan(1.6 / horDis));
            return degree;
        } else {
            return nowDegree;
        }
    }

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
        SmartDashboard.putNumber("LimelightX", getTx());
        SmartDashboard.putNumber("LimelightY", getTy());
        SmartDashboard.putNumber("LimelightID", getTID());
        SmartDashboard.putNumber("latency", getTl());

        SmartDashboard.putNumber("hor_Dis", getHorizontalDisByRoll());
        SmartDashboard.putNumber("MyDistance", getDistance());
    }

}
