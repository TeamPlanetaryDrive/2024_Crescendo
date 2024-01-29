package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
    private final double CAMERA_HEIGHT_METERS;

    private final double CAMERA_PITCH_RADIANS;

    private PhotonCamera camera;

    public PhotonVision(double cameraHeightInches, double cameraPitchDegrees, String photonCameraName) {
        CAMERA_HEIGHT_METERS = Units.inchesToMeters(cameraHeightInches);
        CAMERA_PITCH_RADIANS = Units.degreesToRadians(cameraPitchDegrees);
        camera = new PhotonCamera(photonCameraName);
    }

    /**
    * Returns the distance to the nearest AprilTag target within view of the camera
    * 
    * @return the distance to the nearest AprilTag target, -1 if there are no AprilTags detected
    */
    public double getDistanceToTargetMeters(int aprilTagID1, int aprilTagID2, double targetHeight) {
        //Ids for the center apriltags on the speakers ones are 4 (red) and 7 (blue)

        var result = camera.getLatestResult();

        if(result.hasTargets()) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            PhotonTrackedTarget desiredTarget = null;

            for(PhotonTrackedTarget target : targets) {
                if(target.getFiducialId() == aprilTagID1 || target.getFiducialId() == aprilTagID2) {
                    desiredTarget = target;
                    break;
                }
            }

            if(null == desiredTarget) return -1;

            double range = PhotonUtils.calculateDistanceToTargetMeters(
                CAMERA_HEIGHT_METERS,
                targetHeight,
                CAMERA_PITCH_RADIANS,
                desiredTarget.getPitch()
            );

            return range;
        }
        return -1;
    }

    public double getYaw(int aprilTagID1, int aprilTagID2) {
        var result = camera.getLatestResult();

        if(result.hasTargets()) {
            List<PhotonTrackedTarget> targets = result.getTargets();
            PhotonTrackedTarget desiredTarget = null;

            for(PhotonTrackedTarget target : targets) {
                if(target.getFiducialId() == aprilTagID1 || target.getFiducialId() == aprilTagID2) {
                    desiredTarget = target;
                    break;
                }
            }

            if(null == desiredTarget) return -1;

            return desiredTarget.getYaw();
        }
        return Integer.MIN_VALUE;
    }
}