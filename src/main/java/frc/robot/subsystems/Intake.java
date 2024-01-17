package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private Victor leftIntakeMotor;
    private Victor rightIntakeMotor;
    private final double speed = .5;

    public Intake(int leftIntakeChannel, int rightIntakeChannel) {
        leftIntakeMotor = new Victor(leftIntakeChannel);
        rightIntakeMotor = new Victor(rightIntakeChannel);
    }

    public void intake() {
        leftIntakeMotor.set(speed);
        rightIntakeMotor.set(speed);
    }

    public void stop() {
        leftIntakeMotor.set(0);
        rightIntakeMotor.set(0);
    }
}
