package frc.robot.commands.lift;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lift;

public class LiftExtend extends Command { 
    private Lift lift;

    public LiftExtend(Lift lift) {
        this.lift = lift;
        addRequirements(this.lift);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        lift.extend();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        lift.stop();
    }
}
