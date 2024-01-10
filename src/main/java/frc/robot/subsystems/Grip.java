public class Grip extends SubsystemBase {
    private Victor gripMotor;
    private int motorSpeed; //This should always between -1 and 1

    public Grip(int portNumber) {
         gripMotor = new Victor(portNumber);
         gripMotor.set(0);
    }

    public Grip(int portNumber, int speedOfMotor) {
        gripMotor = new Victor(portNumber);
        motorSpeed = speedOfMotor;
        gripMotor.set(speedOfMotor);
   }

    public void openGripMotor() {
        gripMotor.set(motorSpeed);
    }

    public void closeGripMotor() {
        gripMotor.set(-motorSpeed);
    }

    public void stopGripMotor() {
        gripMotor.set(0);
    }

    public void setMotorSpeed(int motorSpeed) {
        gripMotor.set(0);
    }

    public void stopGripMotor() {
        gripMotor.set(0);
    }
}
