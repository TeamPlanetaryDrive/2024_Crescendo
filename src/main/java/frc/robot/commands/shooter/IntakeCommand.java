package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class IntakeCommand extends Command {

    private Shooter shooter;

    public IntakeCommand(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.init("IntakeCommand");
    }

    @Override
    public void execute() {
        shooter.intake();
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
