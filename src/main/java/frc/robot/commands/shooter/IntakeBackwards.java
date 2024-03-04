package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntake;

public class IntakeBackwards extends Command {
    private ShooterIntake shooter;
    public IntakeBackwards(ShooterIntake shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        shooter.intakeBackwards();
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
