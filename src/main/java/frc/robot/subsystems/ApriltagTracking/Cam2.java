package frc.robot.subsystems.apriltagTracking;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class Cam2 extends SubsystemBase {
    /** Creates a new VisionTrackingPhotovision. */

    private double m_latestLatency;
    public AprilTagFieldLayout aprilTagFieldLayout;

    private final boolean tag = true;
    private final String cameraName = "TagCamera";
    private final PhotonCamera tagCamera;

    public final double cameraHeight = 0.36;
    public final double cameraWeight = 0.0; // I'm still quite confused abt the meaning of having this
    public final double pitchDegree = 15; // 90 - cam_offset
    public final double yawDegree = 0;

    public PhotonPipelineResult results;
    public Transform3d robotToCam = VisionConstants.krobottocam;
    public AprilTagFieldLayout m_layout;
    // Construct PhotonPoseEstimator
    public PhotonPoseEstimator photonPoseEstimator;

    public double distance;
    public double yaw;
    public double pitch;
    public double x;
    public double y;
    public double ID;
    public boolean hasTarget;

    public Cam2() {
        tagCamera = new PhotonCamera(cameraName);
        tagCamera.setPipelineIndex(0);
        tagCamera.setDriverMode(false);
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
                PoseStrategy.CLOSEST_TO_REFERENCE_POSE, tagCamera, robotToCam);
        try {
            m_layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        } catch (IOException err) {
            throw new RuntimeException();
        }

    }

    /**
     * Returns pipeline result
     * 
     * @return (@link PhotonPipelineResult} results
     */
    public PhotonPipelineResult getPipelineResult() {
        results = tagCamera.getLatestResult();
        return results;
    }

    /**
     * Whether the tag's detected
     * 
     * @return hasTarget
     */
    public boolean hasTarget() {
        hasTarget = getPipelineResult().hasTargets();
        return hasTarget;
    }

    /**
     * Get Best Target according to the pipeline.
     * 
     * @return {@link PhotonTrackedTarget} best target
     */
    public PhotonTrackedTarget getBestTarget() {
        results = getPipelineResult();
        m_latestLatency = results.getLatencyMillis() / 1000.;
        PhotonTrackedTarget target = hasTarget()?results.getBestTarget():null;
        return target;
    }

    /**
     * Get a list of tracked tags according to the pipeline.
     * 
     * @return a list of tracked {@link PhotonTrackedTarget} tags
     */
    public List<PhotonTrackedTarget> getTargets() {
        // results = getPipelineResult();
        List<PhotonTrackedTarget> tags = getPipelineResult().getTargets();
        return tags;
    }

    /**
     * Return the last tag of the getTargets() list
     * 
     * @return {@link PhotontrackedTarget} last tag
     */
    public PhotonTrackedTarget getLastTag() {
        return getTargets().get(0);
    }

    /**
     * Returns an array of tag information. Returns zero if nothing detected. 
     * @return [0]: ID; [1]: range; [2]: yaw; [3]: pitch; [4] area; 
     */
    public double[] getTagInfo2() {
        double[] tagInfo = new double[5];
        PhotonTrackedTarget target = getBestTarget();
        ID = target.getFiducialId();
        distance = PhotonUtils.calculateDistanceToTargetMeters(
                cameraHeight, // height
                cameraHeight * (1 / Math.tan(Math.toRadians(target.getPitch() + pitchDegree))),
                Math.toRadians(target.getPitch()),
                Units.degreesToRadians(results.getBestTarget().getPitch()));
        double yaw = target.getYaw();
        double pitch = target.getPitch();
        double area = target.getArea();
        // Transform3d pose = target.getBestCameraToTarget();
        // List<TargetCorner> corners = target.getDetectedCorners();
        tagInfo[0] = hasTarget() ? ID : 0;
        tagInfo[1] = hasTarget() ? distance : 0;
        tagInfo[2] = hasTarget() ? yaw : 0;
        tagInfo[3] = hasTarget() ? pitch : 0;
        tagInfo[4] = hasTarget() ? area : 0;
        return tagInfo;
    }
    
    public int getTagID() {
        Optional<Integer> ID = Optional.of(Integer.valueOf(getBestTarget().getFiducialId()));
        int id = ID.isPresent() ? ID.get() : -1;
        return id;
    }


    /**
     * Get best transform that maps camera space (X = forward, Y = left, Z = up) to
     * object/fiducial tag space (X forward, Y left, Z up) with the lowest
     * reprojection error
     * 
     * @return {@link Transform3d} best cam to tag
     */
    public Transform3d getBestTagTransform3d() {
        Transform3d pose = getBestTarget().getBestCameraToTarget();
        return pose;
    }

    public Pose3d getBotPose() {
        if (getBestTarget() != null) {
            Transform3d camToTag = getBestTarget().getBestCameraToTarget();
            Optional<Pose3d> tag_Pose3d = m_layout.getTagPose(getTagID());
            // Optional<Integer> num = getNum();
            if (tag_Pose3d.isPresent()) {
                Pose3d robotPose3d = PhotonUtils.estimateFieldToRobotAprilTag(camToTag, tag_Pose3d.get(), robotToCam);
                return robotPose3d;
            }
        }
        return new Pose3d();
    }

    // used in facePhoton() method in drivebase
    public Rotation2d getYawToPoseRotation2d(Pose2d robotPose, Pose2d targetPose) {
        Rotation2d targetYaw = PhotonUtils.getYawToPose(robotPose, targetPose);
        return targetYaw;
    }

    public Pose2d getLatestEstimatedRobotPose() {

        if (hasTarget()) {
            Transform3d cameraToTarget = getBestTarget().getBestCameraToTarget();

            Optional<Pose3d> tagPose = m_layout.getTagPose(getBestTarget().getFiducialId());
            // Use Optional so we don't need to do null check

            Transform3d camToRobot = new Transform3d();

            if (tagPose.isPresent()) {
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(cameraToTarget, tagPose.get(), camToRobot);
                return robotPose.toPose2d();
            }
        }
        return new Pose2d();
    }

    /**
     * Gets the pose of a target.
     * 
     * @param robotPose The current robot pose.
     * @param offset    The offset of the desired pose from the target. Positive is
     *                  backwards (X) and right (Y).
     * @return The pose of the specified offset from the target.
     */
    public Pose2d getTargetPose(Pose2d robotPose, Transform3d offset) {
        PhotonTrackedTarget target = getBestTarget();

        if (target != null) {
            Transform3d cameraToTarget = target.getBestCameraToTarget();

            Transform3d targetOffset = cameraToTarget.plus(offset);

            Pose3d pose = new Pose3d(robotPose);
            // Constructs a 3D pose from a 2D pose in the X-Y plane

            Pose3d scoringPose = pose.plus(targetOffset);

            // WARNING: The following code is scuffed. Please proceed with caution.
            Pose2d newPose = scoringPose.toPose2d();

            Rotation2d newRotation = Rotation2d.fromDegrees(newPose.getRotation().getDegrees() - 180.);

            Pose2d finalPose = new Pose2d(newPose.getTranslation(), newRotation).plus(
                    new Transform2d(
                            VisionConstants.krobottocam.getTranslation().toTranslation2d(),
                            VisionConstants.krobottocam.getRotation().toRotation2d()));
            return finalPose;
        }

        return robotPose;
    }

    public double getLatestLatency() {
        return m_latestLatency;
    }

    /**
     * Gets the best tag's pose in 2 dimension
     * 
     * @return best tagPose
     */
    public Pose2d getTagPose2d() {
        if (hasTarget()) {
            Optional<Pose3d> tag_Pose3d = m_layout.getTagPose(getTagID());
            Pose2d tagPose2d = tag_Pose3d.isPresent() ? tag_Pose3d.get().toPose2d() : new Pose2d();
            return tagPose2d;
        } else {
            return new Pose2d();
        }
    }

    /**
     * Get the pose of the desired tag in 2 dimension
     * 
     * @param index tag ID
     * @return best tagPose
     */
    public Pose2d getDesiredTagPose(int index) {
        if (hasTarget()) {
            Optional<Pose3d> tag_Pose3d = m_layout.getTagPose(index);
            Pose2d tagPose2d = tag_Pose3d.isPresent() ? tag_Pose3d.get().toPose2d() : new Pose2d();
            return tagPose2d;
        } else {
            return new Pose2d();
        }
    }

    public void putDashboard() {
        SmartDashboard.putNumber("range", getTagInfo2()[1]);
        SmartDashboard.putNumber("ID", getTagInfo2()[0]);
        SmartDashboard.putNumber("yaw", getTagInfo2()[2]);
        SmartDashboard.putNumber("pitch", getTagInfo2()[3]);
        SmartDashboard.putNumber("area", getTagInfo2()[4]);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}