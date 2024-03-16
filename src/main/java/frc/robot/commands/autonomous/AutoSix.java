package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.auxiliarycommands.DriveBack;
import frc.robot.commands.autonomous.auxiliarycommands.DriveForward;
import frc.robot.commands.shooter.Intake;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterIntake;

public class AutoSix extends SequentialCommandGroup{
    public AutoSix(DriveTrain drive, ShooterIntake shooter) {
        addRequirements(shooter, drive);
        addCommands(
            new Shoot(shooter).withTimeout(3),
            new ParallelCommandGroup(
                new DriveBack(drive),
                new Intake(shooter)
            ).withTimeout(4),
            new ParallelCommandGroup(
                new DriveForward(drive),
                new Intake(shooter).withTimeout(1.5)
            ).withTimeout(2.5),
            new Shoot(shooter).withTimeout(4.5)
        );
    }
}
