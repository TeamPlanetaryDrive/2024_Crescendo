package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class LiftCommand extends Command {

    private Shooter shooter;

    public LiftCommand(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.init("LiftCommand");
    }

    @Override
    public void execute() {
        shooter.toggleLift();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAllMotors();
    }
}
