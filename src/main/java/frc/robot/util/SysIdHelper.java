package frc.robot.util;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

public class SysIdHelper {
    private SysIdRoutine routine;

    public SysIdHelper() {
        routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(Robot.drive::driveVoltsSysID, 
            log -> {
                log.motor("drive-left")
            }, 
            Robot.drive));
    }
}
