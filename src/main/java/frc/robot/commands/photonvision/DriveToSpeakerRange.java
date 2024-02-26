package frc.robot.commands.photonvision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class DriveToSpeakerRange extends Command {
    private PhotonVision photonVision;
    private DriveTrain drive;

    private final int APRIL_TAG_ID_SPEAKER_BLUE = Constants.kAPRIL_TAG_ID_SPEAKER_BLUE;
    private final int APRIL_TAG_ID_SPEAKER_RED = Constants.kAPRIL_TAG_ID_SPEAKER_RED;
    private final double SHOOTING_DISTANCE_TO_SPEAKER_METERS = Constants.kSHOOTING_DISTANCE_TO_SPEAKER_METERS;
    private final double SPEAKER_HEIGHT_METERS = Constants.kSPEAKER_HEIGHT_METERS;

    private double speed;
    private double range;

    public DriveToSpeakerRange(PhotonVision photonVision, DriveTrain drive) {
        this.photonVision = photonVision;
        this.drive = drive;
        addRequirements(this.photonVision, this.drive);
    }

    @Override
    public void initialize() {
        range = photonVision.getDistanceToTargetMeters(APRIL_TAG_ID_SPEAKER_BLUE, APRIL_TAG_ID_SPEAKER_RED, SPEAKER_HEIGHT_METERS);
        if(range != -1) {
            speed = .5;
        }
        if(range < SHOOTING_DISTANCE_TO_SPEAKER_METERS) {
            range *= -1;
        }
        drive.resetEncoders();
    }

    @Override
    public void execute() {
        drive.arcadeDrive(speed, 0);
    }

    @Override
    public boolean isFinished() {
        return range == -1 || Math.abs(drive.getAverageDistance()) > SHOOTING_DISTANCE_TO_SPEAKER_METERS;
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
}
