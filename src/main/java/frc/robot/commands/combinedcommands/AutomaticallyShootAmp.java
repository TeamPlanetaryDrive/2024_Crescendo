package frc.robot.commands.combinedcommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.photonvision.DriveToAmpRange;
import frc.robot.commands.photonvision.TurnToAmp;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShooterDown;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterIntake;

public class AutomaticallyShootAmp extends SequentialCommandGroup {
    public AutomaticallyShootAmp(PhotonVision photonVision, DriveTrain drive, ShooterIntake shooter) {
        addRequirements(photonVision, drive, shooter);
        addCommands(
            new ParallelCommandGroup(
                new ShooterDown(shooter),
                new TurnToAmp(photonVision, drive)
            ).withTimeout(3),
            new DriveToAmpRange(photonVision, drive).withTimeout(4),
            new Shoot(shooter).withTimeout(3)
        );
    }
}
