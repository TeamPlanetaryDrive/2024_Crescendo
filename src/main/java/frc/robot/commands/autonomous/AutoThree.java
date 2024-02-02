package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autonomous.auxiliarycommands.DriveBack;
import frc.robot.commands.autonomous.auxiliarycommands.Turn;
import frc.robot.commands.shooter.AutomaticallyShootAmp;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterIntake;

public class AutoThree extends SequentialCommandGroup {
    public AutoThree(PhotonVision photonVision, DriveTrain drive, ShooterIntake shooter) {
        addRequirements(photonVision, drive, shooter);

        try {
            DriverStation.Alliance alliance = DriverStation.getAlliance().get();
            if(alliance == DriverStation.Alliance.Blue) {
                addCommands(
                    new DriveBack(drive).withTimeout(1.25),
                    new Turn(drive, true, 90).withTimeout(2),
                    new AutomaticallyShootAmp(photonVision, drive, shooter)
                );
            }
            else if(alliance == DriverStation.Alliance.Red) {
                addCommands(
                    new DriveBack(drive).withTimeout(1.25),
                    new Turn(drive, false, 90).withTimeout(2),
                    new AutomaticallyShootAmp(photonVision, drive, shooter)
                );
            }
            else {
                addCommands(new DriveBack(drive).withTimeout(1.25));
            }
        }
        catch(Exception e) {
            addCommands(new DriveBack(drive).withTimeout(1.25));
        }
    }
}
