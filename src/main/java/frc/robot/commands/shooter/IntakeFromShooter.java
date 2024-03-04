package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntake;

public class IntakeFromShooter extends Command {
    private ShooterIntake shooter;
    public IntakeFromShooter(ShooterIntake shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.intakeFromShooters();
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
