package frc.robot.commands.drivetraincommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends Command {
    private DriveTrain drive;
    private int driveMode;
    private SlewRateLimiter lAccelLimiter, rAccelLimiter;

    private final double MAX_ACCEL_METERS_PER_SECOND = 3.5;
    
    public DriveCommand(DriveTrain drive, int driveMode) {
        this.drive = drive;
        this.driveMode = driveMode;
        lAccelLimiter = new SlewRateLimiter(MAX_ACCEL_METERS_PER_SECOND);
        rAccelLimiter = new SlewRateLimiter(MAX_ACCEL_METERS_PER_SECOND);
        addRequirements(this.drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(driveMode == 1) {
            double left = -RobotMap.XController.getLeftY();
            double right = -RobotMap.XController.getRightY();

            if(Math.round(left * 8) != 0 || Math.round(right * 8) != 0)
                drive.tankDrive(lAccelLimiter.calculate(left), rAccelLimiter.calculate(right));
        } else {
            double yax = -RobotMap.XController.getLeftY();
            double xax = -RobotMap.XController.getLeftX();
            if(Math.round(yax * 8) != 0 || Math.round(xax * 8) != 0)
                drive.arcadeDrive(lAccelLimiter.calculate(yax), xax);
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(driveMode == 1) {
            drive.tankDrive(0, 0);
        }
        else if(driveMode == 0) {
            drive.arcadeDrive(0, 0);
        }
        else {
            drive.tankDrive(0, 0);
            drive.arcadeDrive(0, 0);
        }
    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
