package frc.robot.commands.combinedcommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.photonvision.DriveToSpeakerRange;
import frc.robot.commands.photonvision.TurnToSpeaker;
import frc.robot.commands.shooter.Shoot;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterIntake;

public class AutomaticallyShoot extends SequentialCommandGroup {
    public AutomaticallyShoot(PhotonVision photonVision, DriveTrain drive, ShooterIntake shooter, double distanceToSpeaker, double acceptableError) {
        addRequirements(photonVision, drive, shooter);
        addCommands(
            new TurnToSpeaker(photonVision, drive).withTimeout(3),
            new DriveToSpeakerRange(photonVision, drive, distanceToSpeaker, acceptableError).withTimeout(4),
            new Shoot(shooter).withTimeout(3)
        );
    }
    
}
