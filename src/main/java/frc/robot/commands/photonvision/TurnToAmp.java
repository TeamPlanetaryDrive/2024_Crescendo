package frc.robot.commands.photonvision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class TurnToAmp extends Command{
    private PhotonVision photonVision;
    private DriveTrain drive;

    private final int APRIL_TAG_ID_AMP_BLUE = Constants.kAPRIL_TAG_ID_AMP_BLUE;
    private final int APRIL_TAG_ID_AMP_RED = Constants.kAPRIL_TAG_ID_AMP_RED;

    private double speed;
    private double yaw;

    public TurnToAmp(PhotonVision photonVision, DriveTrain drive) {
        this.photonVision = photonVision;
        this.drive = drive;
        addRequirements(this.photonVision, this.drive);
    }

    @Override
    public void initialize() {
        yaw = photonVision.getYaw(APRIL_TAG_ID_AMP_BLUE, APRIL_TAG_ID_AMP_RED);
        if(yaw != Integer.MIN_VALUE) {
            speed = -Math.signum(yaw)/2.0; // check this too
        }
        drive.resetGyro();
    }

    @Override 
    public void execute() {
        drive.arcadeDrive(0, speed);
    }

    @Override
    public boolean isFinished() {
        return yaw == Integer.MIN_VALUE || Math.round(Math.abs(drive.getAngle())) == Math.abs(yaw) ||
                Math.round(Math.abs(360-drive.getAngle())) == Math.abs(yaw);
    }

    @Override 
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
}
