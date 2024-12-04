/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.Constants;
import frc.robot.commands.drivetraincommands.DriveCommand;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.wpilibj.RobotController;

public class DriveTrain extends SubsystemBase {
  private final DifferentialDrive robotDrive;
  private double kP = 0.68;
  private ShuffleboardTab tab = Shuffleboard.getTab("Tab 3");
  private GenericEntry maxSpeed = tab.add("P", 0.0).getEntry();
  private final double kI = 0.0;
  private final double kD = 0.0;
  private Victor lMotor, rMotor;
  private Encoder lEncoder, rEncoder;
  private ADXRS450_Gyro angleGyro;
  private int driveMode = 0; //0: Arcade, 1: Tank
  private DifferentialDriveOdometry odometry;

  private Trajectory autoTrajectory;

  private final double LEFT_METERS_PER_PULSE = Constants.kLEFT_ENCODER_METERS_PER_PULSE;
  private final double RIGHT_METERS_PER_PULSE = Constants.kRIGHT_ENCODER_METERS_PER_PULSE;

  private final MutableMeasure<Voltage> appliedVoltage = MutableMeasure.mutable(Volts.of(0));
  private final MutableMeasure<Distance> distance = MutableMeasure.mutable(Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> velocity = MutableMeasure.mutable(MetersPerSecond.of(0));

  private PIDController pidController;

  SysIdRoutine routine = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
              (Measure<Voltage> volts) -> {
                lMotor.setVoltage(volts.in(Volts));
                rMotor.setVoltage(volts.in(Volts));
              }, 
            log -> {
                log.motor("drive-left")
                    .voltage(
                      appliedVoltage.mut_replace(lMotor.get() * RobotController.getBatteryVoltage(), Volts))
                    .linearPosition(distance.mut_replace(lEncoder.getDistance(), Meters))
                    .linearVelocity(velocity.mut_replace(lEncoder.getRate(), MetersPerSecond));
                log.motor("drive-right")
                  .voltage(
                    appliedVoltage.mut_replace(rMotor.get() * RobotController.getBatteryVoltage(), Volts))
                  .linearPosition(distance.mut_replace(rEncoder.getDistance(), Meters))
                  .linearVelocity(velocity.mut_replace(rEncoder.getRate(), MetersPerSecond));
            }, 
            this));

  public DriveTrain(int leftMotor, int leftMotor2, int rightMotor, int rightMotor2, int[] leftEncoder, int[] rightEncoder) {
    super();
    lMotor = new Victor(leftMotor);
    lMotor.addFollower(new Victor(leftMotor2));
    rMotor = new Victor(rightMotor);
    rMotor.addFollower(new Victor(rightMotor2));
    rMotor.setInverted(true);
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
    
    pidController = new PIDController(kP, kI, kD);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
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
    // System.out.println("tank driving");
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
    return new DriveCommand(this, driveMode, robotDrive);
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
          new Translation2d(1, 0)
        ),
        new Pose2d(0, 0, new Rotation2d(0)),
        config
      );
    }

    autoTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
          new Translation2d(1, 0)
        ),
        new Pose2d(0, 0, new Rotation2d(0)),
        config
      );

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

  public void travelXFeet(double x) {
    pidController.setP(maxSpeed.getDouble(0.0));
    double y = Math.max(pidController.calculate(getAverageDistance(), -1), -0.7);
    SmartDashboard.putNumber("Speed", y);
    arcadeDrive(y, 0);
  }
}
