package frc.robot.commands.photonvision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class DriveToAmpRange extends Command {
    private PhotonVision photonVision;
    private DriveTrain drive;

    private final int APRIL_TAG_ID_AMP_BLUE = Constants.kAPRIL_TAG_ID_AMP_BLUE;
    private final int APRIL_TAG_ID_AMP_RED = Constants.kAPRIL_TAG_ID_AMP_RED;
    private final double SHOOTING_DISTANCE_TO_AMP_FEET = Constants.kSHOOTING_DISTANCE_TO_AMP_FEET;
    private final double AMP_HEIGHT_METERS = Constants.kAMP_HEIGHT_METERS;

    private double range;
    private double speed;

    public DriveToAmpRange(PhotonVision photonVision, DriveTrain drive) {
        this.photonVision = photonVision;
        this.drive = drive;
        addRequirements(this.photonVision, this.drive);
    }

    @Override
    public void initialize() {
        range = photonVision.getDistanceToTargetMeters(APRIL_TAG_ID_AMP_RED, APRIL_TAG_ID_AMP_BLUE, AMP_HEIGHT_METERS);
        if(range != -1) {
            speed = .5;
        }
        range = Units.metersToFeet(range);
        if(range < SHOOTING_DISTANCE_TO_AMP_FEET) {
            speed *= -1;
        }
        drive.resetEncoders();
    }

    @Override
    public void execute() {
        drive.arcadeDrive(speed, 0);
    }

    @Override
    public boolean isFinished() {
        return range == -1 || Math.abs(drive.getAverageDistance()) > SHOOTING_DISTANCE_TO_AMP_FEET;
    }

    @Override
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
}
