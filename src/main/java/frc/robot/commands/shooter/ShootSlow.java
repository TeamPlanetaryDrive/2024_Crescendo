package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntake;

public class ShootSlow extends Command {
    private ShooterIntake shooter;
    public ShootSlow(ShooterIntake shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.shootSlow();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean isFinished) {
        shooter.stopMotors();
    }
}
