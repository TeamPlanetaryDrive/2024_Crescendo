package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{

    // Keep the "Physical" Components of the Subsystems inside of the Subsystem as a Private Instance Variable
    private DoubleSolenoid solenoid;

    public Arm(int solenoidChannelLeft, int solenoidChannelRight) {
        solenoid = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, solenoidChannelLeft, solenoidChannelRight);

        solenoid.set(Value.kReverse);
    }

    public void extendArm() {
        solenoid.set(Value.kForward);
    }

    public void closeArm() {
        solenoid.set(Value.kReverse);
    }

    public void toggle() {
        solenoid.toggle();
    }
}
