package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntake;

public class Intake extends Command {
    private ShooterIntake shooter;
    public Intake(ShooterIntake shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        shooter.intake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
    @Override
    public void end(boolean isFinished) {
        shooter.stopIntakeMotors();
    }
}
