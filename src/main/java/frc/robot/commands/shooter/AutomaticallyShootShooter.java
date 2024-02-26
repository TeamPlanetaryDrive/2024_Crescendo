package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.photonvision.DriveToSpeakerRange;
import frc.robot.commands.photonvision.TurnToSpeaker;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterIntake;

public class AutomaticallyShootShooter extends SequentialCommandGroup {
    public AutomaticallyShootShooter(PhotonVision photonVision, DriveTrain drive, ShooterIntake shooter) {
        addRequirements(photonVision, drive, shooter);
        if(photonVision.ensureValidIDInSight(4, 7)) {
            addCommands(
                new TurnToSpeaker(photonVision, drive).withTimeout(3),
                new DriveToSpeakerRange(photonVision, drive).withTimeout(4),
                new Shoot(shooter).withTimeout(3)
            );
        }
        else {
            System.out.println("No valid IDs in sight");
        }
    }
    
}
