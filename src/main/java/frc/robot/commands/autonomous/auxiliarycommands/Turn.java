package frc.robot.commands.autonomous.auxiliarycommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class Turn extends Command {
    private DriveTrain drive;
    private int multiplier;
    private double angleDegrees;

    public Turn(DriveTrain drive, boolean right, double angleDegrees) {
        this.drive = drive;
        multiplier = right ? -1 : 1;
        this.angleDegrees = angleDegrees;
        addRequirements(this.drive);
    }

    public void initialize() {
        drive.resetGyro();
    }

    public void execute() {
        drive.arcadeDrive(0, multiplier * .5);
    }

    public boolean isFinished() {
        return Math.round(Math.abs(drive.getAngle())) == Math.abs(angleDegrees)
                || Math.round(Math.abs(360-drive.getAngle())) == Math.abs(angleDegrees);
    }

    public void end(boolean interrupted) {
        drive.arcadeDrive(0, 0);
    }
}
