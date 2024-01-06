package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SubsystemExample extends SubsystemBase{
    private Victor motor;
    private DoubleSolenoid sole;
    
    public SubsystemExample(int motorChannel, int solenoidChannelLeft, int solenoidChannelRight) {
        motor = new Victor(motorChannel);
        sole = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM, solenoidChannelLeft, solenoidChannelRight);

        sole.set(Value.kReverse);
    }

    public void doSomething() {
        motor.set(.50);
        sole.toggle();
    }

    public void stop() {
        motor.set(0);
    }

    //This calls every schedule run
    @Override
    public void periodic() {}
}
