package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.auxiliarycommands.DriveBack;
import frc.robot.commands.autonomous.auxiliarycommands.Turn;
import frc.robot.commands.combinedcommands.AutomaticallyShootAmp;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterIntake;

public class AutoThree extends SequentialCommandGroup {
    public AutoThree(PhotonVision photonVision, DriveTrain drive, ShooterIntake shooter, double distanceToSpeaker, double acceptableError) {
        addRequirements(photonVision, drive, shooter);

        try {
            DriverStation.Alliance alliance = DriverStation.getAlliance().get();
            if(alliance == DriverStation.Alliance.Blue) {
                addCommands(
                    new DriveBack(drive),
                    new Turn(drive, true),
                    new AutomaticallyShootAmp(photonVision, drive, shooter, distanceToSpeaker, acceptableError)
                );
            }
            else if(alliance == DriverStation.Alliance.Red) {
                addCommands(
                    new DriveBack(drive),
                    new Turn(drive, false),
                    new AutomaticallyShootAmp(photonVision, drive, shooter, distanceToSpeaker, acceptableError)
                );
            }
            else {
                addCommands(new DriveBack(drive));
            }
        }
        catch(Exception e) {
            addCommands(new DriveBack(drive));
        }
    }
}
