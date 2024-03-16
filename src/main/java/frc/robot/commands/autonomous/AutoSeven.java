package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.auxiliarycommands.DriveBack;
import frc.robot.commands.autonomous.auxiliarycommands.DriveForward;
import frc.robot.commands.shooter.Intake;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShooterIntake;

public class AutoSeven extends SequentialCommandGroup{
    public AutoSeven(ShooterIntake shooter) {
        addRequirements(shooter);
        addCommands(
            new Shoot(shooter).withTimeout(5)
        );
    }
}
