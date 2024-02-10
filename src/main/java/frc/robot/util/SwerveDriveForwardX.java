package frc.robot.util;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.swervedrive.SwerveSubsystem;
import frc.robot.util.swervedrive.Constants.DriveConstants;

public class SwerveDriveForwardX extends Command {
    private float speed;
    private SwerveSubsystem swerveSubsystem;
    private SwerveModuleState[] swerveModuleStates;
    public SwerveDriveForwardX(SwerveSubsystem swerveSubsystem, float speed, boolean isForwards, double distanceToTravel) {

        if (isForwards) {
            this.speed = speed;
        } else {
            this.speed = -speed;
        }

        //Below line of code may work!
        /// @code this.speed = (isForwards)?speed:-speed;

        this.swerveSubsystem = swerveSubsystem;
        swerveModuleStates = new SwerveModuleState[4];

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speed, 0, 0);
        swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        swerveSubsystem.setModuleStates(swerveModuleStates);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        //Figure out using encoder
        
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }
}
