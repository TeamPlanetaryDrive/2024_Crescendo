package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntake;

public class ShooterUp extends Command {
    private ShooterIntake shooter;
    public ShooterUp(ShooterIntake shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.liftUp();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
    
    @Override
    public void end(boolean isFinished) {
    }
}
