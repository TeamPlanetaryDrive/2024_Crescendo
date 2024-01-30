package frc.robot.commands.photonvision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class DriveToAmpRange extends Command {
    private PhotonVision photonVision;
    private DriveTrain drive;

    private final int APRIL_TAG_ID_AMP_BLUE = 6;
    private final int APRIL_TAG_ID_AMP_RED = 5;
    private final double[] ACCEPTABLE_DISTANCES;
    private final double AMP_HEIGHT_METERS = Units.inchesToMeters(48.125);

    private double speed;
    private double currentRange;

    public DriveToAmpRange(PhotonVision photonVision, DriveTrain drive, double distanceToSpeakerMeters, double acceptableErrorMeters) {
        this.photonVision = photonVision;
        this.drive = drive;
        ACCEPTABLE_DISTANCES = new double[] {distanceToSpeakerMeters-acceptableErrorMeters, distanceToSpeakerMeters+acceptableErrorMeters};
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        currentRange = photonVision.getDistanceToTargetMeters(APRIL_TAG_ID_AMP_BLUE, APRIL_TAG_ID_AMP_RED, AMP_HEIGHT_METERS);
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
