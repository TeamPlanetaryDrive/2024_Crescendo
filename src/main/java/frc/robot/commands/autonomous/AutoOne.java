package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.auxiliarycommands.DriveBack;
import frc.robot.subsystems.DriveTrain;

public class AutoOne extends SequentialCommandGroup{
    public AutoOne(DriveTrain drive) {
        addRequirements(drive);

        addCommands(
            new DriveBack(drive)
        );
    }
}
