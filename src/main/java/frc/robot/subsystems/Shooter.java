package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Shooter extends SubsystemBase{
    private final Victor[] mainMotors;
    private final Victor[] storageMotors;
    private final Timer timer;
    private final DoubleSolenoid shooterLift;

    private final double SHOOTING_SPEED = 1;
    private final double INTAKE_SPEED = -1;
    private final double SLOW_INTAKE_SPEED = -.65;

    public Shooter(int[] main, int[] storage, int[] lift) {
        timer = new Timer();

        mainMotors = new Victor[] {
            new Victor(main[0]),
            new Victor(main[1])
        };

        storageMotors = new Victor[] {
            new Victor(storage[0]),
            new Victor(storage[1])
        };

        shooterLift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, lift[0], lift[1]);
        shooterLift.set(Value.kReverse);
    }

    public void init(String location) {
        Robot.logger.log("[" + location + "] Initializing Shooter Subsystem...");
    }

    public void shoot() {
        mainMotors[0].set(SHOOTING_SPEED);
        mainMotors[1].set(SHOOTING_SPEED);
        timer.reset();
        timer.start();
        while(timer.get() <= 1) {}
        timer.stop();
        storageMotors[0].set(SHOOTING_SPEED);
        storageMotors[1].set(SHOOTING_SPEED);
    }

    public void intake() {
        mainMotors[0].set(INTAKE_SPEED);
        mainMotors[1].set(INTAKE_SPEED);
        storageMotors[0].set(SLOW_INTAKE_SPEED);
        storageMotors[1].set(SLOW_INTAKE_SPEED);
    }

    public void toggleLift() {
        shooterLift.toggle();
    }

    public void stopAllMotors() {
        mainMotors[0].set(0);
        mainMotors[1].set(0);
        storageMotors[0].set(0);
        storageMotors[1].set(0);
    }
}
