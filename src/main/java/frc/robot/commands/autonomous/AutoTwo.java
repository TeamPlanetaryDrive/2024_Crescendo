package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.auxiliarycommands.DriveBack;
import frc.robot.commands.shooter.Intake;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterIntake;

public class AutoTwo extends SequentialCommandGroup {
    public AutoTwo(PhotonVision photonVision, DriveTrain drive, ShooterIntake shooter) {
        addRequirements(photonVision, drive, shooter);
        addCommands(
            new ParallelCommandGroup(
                new DriveBack(drive).withTimeout(5),
                new Intake(shooter)
            ).withTimeout(10)
        );
    }
}
