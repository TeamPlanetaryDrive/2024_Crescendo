package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.auxiliarycommands.DriveBackMore;
import frc.robot.commands.shooter.Intake;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterIntake;

public class AutoThree extends SequentialCommandGroup{
    public AutoThree(DriveTrain drive, ShooterIntake shooter) {
        addRequirements(drive, shooter);

        addCommands(
            new Intake(shooter).withTimeout(4),
            new Shoot(shooter).withTimeout(6),
            new ParallelCommandGroup(
                new DriveBackMore(drive),
                new Intake(shooter)
            ).withTimeout(5)
        );
    }
}