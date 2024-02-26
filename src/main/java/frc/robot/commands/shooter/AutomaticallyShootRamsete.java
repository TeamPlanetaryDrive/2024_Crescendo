package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.photonvision.DriveToSpeakerRamsete;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterIntake;

public class AutomaticallyShootRamsete extends SequentialCommandGroup {
    public AutomaticallyShootRamsete(PhotonVision photonVision, DriveTrain drive, ShooterIntake shooter) {
        addRequirements(photonVision, drive, shooter);
        if(photonVision.ensureValidIDInSight(4, 7)) {
            addCommands(
                DriveToSpeakerRamsete.getCommand().withTimeout(5),
                new Shoot(shooter).withTimeout(3)
            );
        }
        else {
            System.out.println("No IDs are valid");
        }
    }
}
