package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterIntake;

public class ShooterDown extends Command {
    private ShooterIntake shooter;
    public ShooterDown(ShooterIntake shooter) {
        this.shooter = shooter;
        addRequirements(this.shooter);
    }

    @Override
    public void initialize() {
        shooter.liftDown();
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
