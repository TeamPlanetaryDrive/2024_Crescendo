package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ShooterIntake extends SubsystemBase{
    private final Victor[] mainMotors;
    private final Victor[] intakeMotors;
    private final DoubleSolenoid shooterLift;
    private final Timer timer;

    private final double SHOOTING_SPEED = 1;
    private final double INTAKE_SPEED = .65;

    public ShooterIntake(int[] main, int[] intake, int[] lift) {
        mainMotors = new Victor[] {
            new Victor(main[0]),
            new Victor(main[1])
        };

        intakeMotors = new Victor[] {
            new Victor(intake[0]),
            new Victor(intake[1])
        };

        shooterLift = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, lift[0], lift[1]);
        shooterLift.set(Value.kReverse);

        timer = new Timer();
    }

    public void init(String location) {
        Robot.logger.log("[" + location + "] Initializing Shooter Subsystem...");
    }

    public void shoot() {
        mainMotors[0].set(SHOOTING_SPEED);
        mainMotors[1].set(SHOOTING_SPEED);
        timer.start();
        while(timer.get() < 2) {}
        timer.stop();
        intake();
    }

    public void intake() {
        intakeMotors[0].set(INTAKE_SPEED);
        intakeMotors[1].set(INTAKE_SPEED);
    }

    public void toggleLift() {
        shooterLift.toggle();
    }

    public void stopShooterMotors() {
        mainMotors[0].set(0);
        mainMotors[1].set(0);
    }

    public void stopIntakeMotors() {
        intakeMotors[0].set(0);
        intakeMotors[1].set(0);
    }

    public void stopMotors() {
        stopShooterMotors();
        stopIntakeMotors();
    }
}
