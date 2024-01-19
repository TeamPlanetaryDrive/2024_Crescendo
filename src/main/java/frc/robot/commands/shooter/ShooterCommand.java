package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterCommand extends Command {

    private Shooter shooter;
    private Timer time;

    public ShooterCommand(Shooter shooter) {
        this.shooter = shooter;

        addRequirements(this.shooter);
    } 

    @Override
    public void initialize() {
        shooter.init("ShooterCommand");
        time.start();
        shooter.shoot();
    }

    @Override
    public void execute() {}

    @Override
    public boolean isFinished() {
        return time.get() >= 2.5;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopAllMotors();
    }
}