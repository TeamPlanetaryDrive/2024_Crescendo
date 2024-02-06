/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.commands.drivetraincommands.DriveCommand;

public class DriveTrain extends SubsystemBase {
  private final DifferentialDrive robotDrive;
  private Victor lMotor, rMotor;
  private Encoder lEncoder, rEncoder;
  private ADXRS450_Gyro angleGyro;
  private int driveMode = 1; //0: Arcade, 1: Tank
  private DifferentialDriveOdometry odometry;

  private Trajectory autoTrajectory;

  private final double LEFT_METERS_PER_PULSE = Units.feetToMeters(Constants.kLEFT_ENCODER_FEET_PER_PULSE);
  private final double RIGHT_METERS_PER_PULSE = Units.feetToMeters(Constants.kRIGHT_ENCODER_FEET_PER_PULSE_FEET);

  public DriveTrain(int leftMotorOne, int leftMotorTwo, int rightMotorOne, int rightMotorTwo, int[] leftEncoder, int[] rightEncoder) {
    super();
    lMotor = new Victor(leftMotorOne);
    lMotor.addFollower(new Victor(leftMotorTwo));

    rMotor = new Victor(rightMotorOne);
    rMotor.addFollower(new Victor(rightMotorTwo));
    robotDrive = new DifferentialDrive(lMotor, rMotor);

    lEncoder = new Encoder(leftEncoder[0], leftEncoder[1]);
    rEncoder = new Encoder(rightEncoder[0], rightEncoder[1]);

    lEncoder.setDistancePerPulse(LEFT_METERS_PER_PULSE); 
    lEncoder.setReverseDirection(true); 
    rEncoder.setDistancePerPulse(RIGHT_METERS_PER_PULSE); 

    resetEncoders();

    angleGyro = new ADXRS450_Gyro();
    angleGyro.calibrate();

    odometry = new DifferentialDriveOdometry(angleGyro.getRotation2d(), lEncoder.getDistance(), rEncoder.getDistance());
  }

  public void setDriveMode(int mode) {
    driveMode = mode;
  }

  //Odometry Functions
  @Override
  public void periodic() {
    odometry.update(angleGyro.getRotation2d(), lEncoder.getDistance(), rEncoder.getDistance());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(lEncoder.getRate(), rEncoder.getRate());
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(
        angleGyro.getRotation2d(), lEncoder.getDistance(), rEncoder.getDistance(), pose);

  }

  //Driving Functions
  public void arcadeDrive(double forward, double rotation) {
    robotDrive.arcadeDrive(forward, rotation);
  }

  public void tankDrive(double left, double right) {
    robotDrive.tankDrive(left, right);
  }

  public void tankDriveVolts(double leftV, double rightV) {
    lMotor.setVoltage(leftV);
    rMotor.setVoltage(rightV);
    robotDrive.feed();
  }

  //Encoder Functions
  public void resetEncoders() {
    lEncoder.reset();
    rEncoder.reset();
  }

  public Encoder getLeftEncoder() {
    return lEncoder;
  }

  public Encoder getRightEncoder() {
    return rEncoder;
  }

  public double getAverageDistance() {
    return (lEncoder.getDistance() + rEncoder.getDistance())/2;
  }

  public void setMaxOutput(double maxOutput) {
    robotDrive.setMaxOutput(maxOutput);
  }

  //Gyro functions
  public void resetGyro() {
    angleGyro.reset();
  }

  public double getAngle() {
    return angleGyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return -angleGyro.getRate();
  }

  //Commands
  public Command getDefaultCommand() {
    return new DriveCommand(this, driveMode);
  }

  public Command getAutonomousCommand() {
    var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
      new SimpleMotorFeedforward(
        Constants.ksVOLTS,
        Constants.kvVOLT_SECONDS_PER_METER,
        Constants.kaVOLT_SECONDS_SQUARED_PER_METER),
        Constants.kDRIVE_KINEMATICS, 
        10);
    
    TrajectoryConfig config = new TrajectoryConfig(
      Constants.kAUTO_MAX_SPEED_METERS_PER_SECOND, 
      Constants.kAUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
      .setKinematics(Constants.kDRIVE_KINEMATICS).addConstraint(autoVoltageConstraint);
    
    
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/Backwards.wpilib.json");
      autoTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch(IOException E) {
      DriverStation.reportError("Unable to open trajectory: paths/Backwards.wpilib.json", E.getStackTrace());
      autoTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
          new Translation2d(-2, 0)
        ),
        new Pose2d(0, 0, new Rotation2d(0)),
        config
      );
    }

    RamseteCommand ramseteCommand = new RamseteCommand(
      autoTrajectory,
      () -> getPose(),
      new RamseteController(Constants.kRAMSETE_B, Constants.kRAMSETE_ZETA),
      new SimpleMotorFeedforward(Constants.ksVOLTS, Constants.kvVOLT_SECONDS_PER_METER, Constants.kaVOLT_SECONDS_SQUARED_PER_METER),
      Constants.kDRIVE_KINEMATICS,
      () -> getWheelSpeeds(),
      new PIDController(Constants.kP_DRIVE_VELOCITY, 0, 0),
      new PIDController(Constants.kP_DRIVE_VELOCITY, 0, 0),
      (left, right) -> tankDriveVolts(left, right),
      this
    );
    
    return Commands.runOnce(() -> this.resetOdometry(autoTrajectory.getInitialPose()))
    .andThen(ramseteCommand).andThen(Commands.runOnce(() -> this.tankDriveVolts(0, 0)));
  }
}
