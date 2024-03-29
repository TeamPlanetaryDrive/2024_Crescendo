package frc.robot.commands.autonomous.auxiliarycommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveBack extends Command {
    private DriveTrain drive;

    public DriveBack(DriveTrain drive) {
        this.drive = drive;
        addRequirements(this.drive);
    }

    public void initialize() {
        drive.resetEncoders();
    }

    public void execute() {
        drive.arcadeDrive(-.67, 0);
    }

    public boolean isFinished() {
        return Math.abs(drive.getAverageDistance()) > .75;
    }

    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
}
