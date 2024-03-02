package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Lift extends SubsystemBase {
    private Spark leftMotor;
    private Spark rightMotor;
    
    private final double LIFT_SPEED = Constants.kLIFT_SPEED;

    public Lift(int leftChannel, int rightChannel) {
        leftMotor = new Spark(leftChannel);
        rightMotor = new Spark(rightChannel);
    }

    public void extend() {
        leftMotor.set(LIFT_SPEED);
        rightMotor.set(LIFT_SPEED);
    } 

    public void retract() {
        leftMotor.set(-LIFT_SPEED);
        rightMotor.set(-LIFT_SPEED);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
}
