package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntake;

public class Shoot extends Command {
    private ShooterIntake shooter;
    public Shoot(ShooterIntake shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.shoot();
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
