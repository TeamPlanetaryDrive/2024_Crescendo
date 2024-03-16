package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.auxiliarycommands.DriveBackMore;
import frc.robot.commands.autonomous.auxiliarycommands.Turn;
import frc.robot.commands.shooter.Intake;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterIntake;

public class AutoFive extends SequentialCommandGroup{
    public AutoFive(DriveTrain drive, ShooterIntake shooter) {
        addRequirements(drive, shooter);

        addCommands(
            new Intake(shooter).withTimeout(3),
            new Shoot(shooter).withTimeout(5),
            new Turn(drive, true, 30).withTimeout(4),
            new DriveBackMore(drive).withTimeout(3)
        );
    }
}