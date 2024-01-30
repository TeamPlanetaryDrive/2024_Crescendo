package frc.robot.commands.autonomous.auxiliarycommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class Turn extends Command {
    private DriveTrain drive;
    private Timer timer;
    private int multiplier;

    public Turn(DriveTrain drive, boolean right) {
        this.drive = drive;
        timer = new Timer();
        if(right) {
            multiplier = -1;
        }
        else {
            multiplier = 1;
        }
        addRequirements(this.drive);
    }

    public void initialize() {
        timer.start();
    }

    public void execute() {
        drive.arcadeDrive(0, multiplier * .5);
    }

    public boolean isFinished() {
        return timer.get() > .75;
    }

    public void end(boolean interrupted) {
        timer.stop();
    }
}
