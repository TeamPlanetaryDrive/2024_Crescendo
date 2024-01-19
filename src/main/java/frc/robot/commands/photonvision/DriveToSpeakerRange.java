package frc.robot.commands.photonvision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class DriveToSpeakerRange extends Command {
    private PhotonVision photonVision;
    private DriveTrain drive;

    private final int APRIL_TAG_ID_SPEAKER_BLUE = 7;
    private final int APRIL_TAG_ID_SPEAKER_RED = 4;
    private final double[] ACCEPTABLE_DISTANCES;
    private final double SPEAKER_HEIGHT_METERS = Units.inchesToMeters(51.875);

    private double speed;
    private double currentRange;

    public DriveToSpeakerRange(PhotonVision photonVision, DriveTrain drive, double distanceToSpeaker, double acceptableError) {
        this.photonVision = photonVision;
        this.drive = drive;
        ACCEPTABLE_DISTANCES = new double[] {distanceToSpeaker-acceptableError, distanceToSpeaker+acceptableError};
    }

    @Override
    public void initialize() {
        double initDistance = photonVision.getDistanceToTargetMeters(APRIL_TAG_ID_SPEAKER_BLUE, APRIL_TAG_ID_SPEAKER_RED, SPEAKER_HEIGHT_METERS);
        if(initDistance != -1) {
            speed = -Math.signum(initDistance)/2;
            drive.arcadeDrive(speed, 0);
        }
    }

    @Override
    public void execute() {
        currentRange = photonVision.getDistanceToTargetMeters(APRIL_TAG_ID_SPEAKER_BLUE, APRIL_TAG_ID_SPEAKER_RED, SPEAKER_HEIGHT_METERS);
        if(currentRange != -1) {
            speed = -Math.signum(currentRange)/2;
            drive.arcadeDrive(speed, 0);
        }
        else {
            drive.arcadeDrive(0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        boolean inRange = currentRange > ACCEPTABLE_DISTANCES[0] && currentRange < ACCEPTABLE_DISTANCES[1];
        return currentRange == -1 || inRange;
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
}
