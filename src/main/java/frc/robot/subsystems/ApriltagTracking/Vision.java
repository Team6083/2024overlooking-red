package frc.robot.subsystems.apriltagTracking;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    private final TagTrackingPhotonvision cam1;
    private final Cam2 cam2;

    public Vision(TagTrackingPhotonvision cam1, Cam2 cam2) {
        this.cam1 = new TagTrackingPhotonvision();
        this.cam2 = new Cam2();
    }



}
