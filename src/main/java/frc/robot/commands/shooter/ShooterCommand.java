package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {

    private Shooter shooter;

    public ShooterCommand(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(this.shooter);
    } 

    @Override
    public void initialize() {
        shooter.init("ShooterCommand");
    }

    @Override
    public void execute() {
        shooter.shoot();
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