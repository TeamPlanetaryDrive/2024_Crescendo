package frc.robot.commands.autonomous.auxiliarycommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class DriveBack extends Command {
    private DriveTrain drive;
    private Timer timer;

    public DriveBack(DriveTrain drive) {
        this.drive = drive;
        timer = new Timer();
        addRequirements(this.drive);
    }

    public void initialize() {
        timer.start();
    }

    public void execute() {
        drive.arcadeDrive(-1, 0);
    }

    public boolean isFinished() {
        return false;
    }

    public void end(boolean interrupted) {
        timer.stop();
    }
}
