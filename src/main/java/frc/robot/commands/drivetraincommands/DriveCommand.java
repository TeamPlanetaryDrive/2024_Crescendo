package frc.robot.commands.drivetraincommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends Command {
    private DriveTrain drive;
    private int driveMode;
    private SlewRateLimiter lAccelLimiter, rAccelLimiter;
    private DifferentialDrive driveTrain;

    private final double maxAccel = Constants.kMAX_ACCELERATION_METERS_PER_SECOND_SQUARED;


    public DriveCommand(DriveTrain drive, int driveMode, DifferentialDrive driveTrain) {
        this.drive = drive;
        this.driveMode = driveMode;
        lAccelLimiter = new SlewRateLimiter(maxAccel);
        rAccelLimiter = new SlewRateLimiter(maxAccel);
        this.driveTrain = driveTrain;
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
                System.out.println("Driving");
        } else {
            double yax = -RobotMap.XController.getLeftY();
            double xax = -RobotMap.XController.getLeftX();
            if(Math.round(yax * 8) != 0 || Math.round(xax * 8) != 0)
                drive.arcadeDrive(lAccelLimiter.calculate(yax), xax);

            System.out.println("y axis: " + yax + " x axis: " + xax);
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
