package frc.robot.commands.drivetraincommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveFiveFeetCommand extends Command {
    private DriveTrain drive;

    public DriveFiveFeetCommand(DriveTrain drive) {
        this.drive = drive;
        addRequirements(this.drive);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Drive Five Feet State", "Initializing");
        drive.resetEncoders();
    }

    @Override
    public void execute() {
        SmartDashboard.putString("Drive Five Feet State", "Executing");
        drive.travelXFeet(5);
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Drive Five Feet State", "Ending");
        drive.arcadeDrive(0, 0);
        drive.resetEncoders();
    }

    @Override 
    public boolean isFinished() {
        SmartDashboard.putString("Distance Traveled", "" + drive.getAverageDistance());
        return Math.abs(drive.getAverageDistance()) > 7.5;
    }
}
