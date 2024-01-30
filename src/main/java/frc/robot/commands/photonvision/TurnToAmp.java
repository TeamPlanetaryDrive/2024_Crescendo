package frc.robot.commands.photonvision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;

public class TurnToAmp extends Command{
    private PhotonVision photonVision;
    private DriveTrain drive;

    private final int APRIL_TAG_ID_AMP_BLUE = 6;
    private final int APRIL_TAG_ID_AMP_RED = 5;

    private final double[] ACCEPTABLE_AIMING_RANGE = {-5., 5.}; //test for later!

    private double speed;
    private double currentYaw;

    public TurnToAmp(PhotonVision photonVision, DriveTrain drive) {
        this.photonVision = photonVision;
        this.drive = drive;

        addRequirements(this.photonVision, this.drive);
    }

    @Override
    public void initialize() {
        
    }

    @Override 
    public void execute() {
        currentYaw = photonVision.getYaw(APRIL_TAG_ID_AMP_BLUE, APRIL_TAG_ID_AMP_RED);
        if(currentYaw != Integer.MIN_VALUE) {
            speed = -Math.signum(currentYaw)/2; // check this too
            drive.arcadeDrive(0, speed);
        }
        else {
            drive.arcadeDrive(0, 0);
        }
    }

    @Override
    public boolean isFinished() {
        boolean acceptableYaw = currentYaw > ACCEPTABLE_AIMING_RANGE[0] && currentYaw < ACCEPTABLE_AIMING_RANGE[1];
        return currentYaw == Integer.MIN_VALUE || acceptableYaw;
    }

    @Override 
    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
}
