package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.auxiliarycommands.DriveBack;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterIntake;

public class AutoOne extends SequentialCommandGroup{
    public AutoOne(DriveTrain drive, ShooterIntake shooter) {
        addRequirements(drive);

        addCommands(
            new Shoot(shooter).withTimeout(5),
            new DriveBack(drive).withTimeout(5)
        );
    }
}