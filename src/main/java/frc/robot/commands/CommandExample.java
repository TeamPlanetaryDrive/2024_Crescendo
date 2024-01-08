package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SubsystemExample;

public class CommandExample extends Command {
    // Make sure to AddRequirements() all Subsystems a Command uses
    private SubsystemExample subsystem;

    public CommandExample(SubsystemExample subsystem) {
        this.subsystem = subsystem;

        addRequirements(this.subsystem);
    } 

    @Override
    public void initialize() {
        subsystem.doSomething();
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        subsystem.stop();
    }
}