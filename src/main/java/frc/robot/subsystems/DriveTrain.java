/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.commands.drivetraincommands.DriveCommand;

public class DriveTrain extends SubsystemBase {
  private final DifferentialDrive robotDrive;
  private Victor lMotor, rMotor;
  private Encoder lEncoder, rEncoder;
  private ADXRS450_Gyro angleGyro;
  private int driveMode = 1; //0: Arcade, 1: Tank

  private final double LEFT_FEET_PER_PULSE = Constants.kLEFT_ENCODER_FEET_PER_PULSE;
  private final double RIGHT_FEET_PER_PULSE = Constants.kRIGHT_ENCODER_FEET_PER_PULSE_FEET;

  public DriveTrain(int leftMotor, int rightMotor, int[] leftEncoder, int[] rightEncoder) {
    super();
    lMotor = new Victor(leftMotor);
    rMotor = new Victor(rightMotor);
    rMotor.setInverted(true);
    robotDrive = new DifferentialDrive(lMotor, rMotor);

    lEncoder = new Encoder(leftEncoder[0], leftEncoder[1]);
    rEncoder = new Encoder(rightEncoder[0], rightEncoder[1]);

    lEncoder.setDistancePerPulse(LEFT_FEET_PER_PULSE); 
    lEncoder.setReverseDirection(true); 
    rEncoder.setDistancePerPulse(RIGHT_FEET_PER_PULSE); 

    angleGyro = new ADXRS450_Gyro();
    angleGyro.calibrate();
  }

  public void setDriveMode(int mode) {
    driveMode = mode;
  }

  public void arcadeDrive(double forward, double rotation) {
    robotDrive.arcadeDrive(forward, rotation);
  }

  public void tankDrive(double left, double right) {
    robotDrive.tankDrive(left, right);
  }

  public void resetEncoders() {
    lEncoder.reset();
    rEncoder.reset();
  }

  public double getAverageDistance() {
    return (lEncoder.getDistance() + rEncoder.getDistance())/2;
  }

  public void resetGyro() {
    angleGyro.reset();
  }

  public double getAngle() {
    return angleGyro.getAngle();
  }

  public Command getDefaultCommand() {
    return new DriveCommand(this, driveMode);
  }
}
