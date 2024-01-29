package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntake;

public class Shoot extends Command {
    private ShooterIntake shooter;
    private Timer timer;
    public Shoot(ShooterIntake shooter) {
        this.shooter = shooter;
        timer = new Timer();
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.shoot();
        timer.start();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return timer.get() > 3.0;
    }
    
    @Override
    public void end(boolean isFinished) {
        shooter.stopMotors();
    }
}
