package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;

public class LiftRetract extends Command { 
    private Lift lift;

    public LiftRetract(Lift lift) {
        this.lift = lift;
        addRequirements(this.lift);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        lift.retract();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        lift.stop();
    }
}
