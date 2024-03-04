package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ShooterIntake extends SubsystemBase{
    private final Talon[] mainMotors;
    private final Spark[] intakeMotors;

    private final double SHOOTING_SPEED = Constants.kSHOOTING_SPEED;
    private final double INTAKE_SPEED = Constants.kINTAKE_SPEED;

    public ShooterIntake(int[] main, int[] intake) {
        mainMotors = new Talon[] {
            new Talon(main[0]),
            new Talon(main[1])
        };

        intakeMotors = new Spark[] {
            new Spark(intake[0]),
            new Spark(intake[1])
        };

        mainMotors[1].setInverted(true);

        intakeMotors[1].setInverted(true);
        intakeMotors[0].setInverted(true);
    }

    public void init(String location) {
        Robot.logger.log("[" + location + "] Initializing Shooter Subsystem...");
    }

    public void shoot() {
        mainMotors[0].set(SHOOTING_SPEED);
        mainMotors[1].set(SHOOTING_SPEED);
        intake();
    }

    public void shootSlow() {
        mainMotors[0].set(SHOOTING_SPEED * .75);
        mainMotors[1].set(SHOOTING_SPEED * .75);
        intake();
    }

    public void intakeFromShooters() {
        mainMotors[0].set(-SHOOTING_SPEED*3.0/4.0);
        mainMotors[1].set(-SHOOTING_SPEED*3.0/4.0);
        intakeBackwards();
    }

    public void intake() {
        intakeMotors[0].set(INTAKE_SPEED);
        intakeMotors[1].set(INTAKE_SPEED);
    }

    public void intakeBackwards() {
        intakeMotors[0].set(-INTAKE_SPEED);
        intakeMotors[1].set(-INTAKE_SPEED);
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
