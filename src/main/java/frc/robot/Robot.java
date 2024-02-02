/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.autonomous.AutoOne;
import frc.robot.commands.autonomous.AutoThree;
import frc.robot.commands.autonomous.AutoTwo;
import frc.robot.subsystems.*;
import frc.robot.util.Logger;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  // Declare bot info
  public static String name = "Wayne's Bot";
  public static Integer year = 2024;

  // Declare bot utilities
  public static Logger logger;

  // Declare Subsystems as Instance Variables
  public static DriveTrain drive;
  public static ShooterIntake shooter;
  public static PhotonVision vision;
  public static Lift lift;

  public static OI m_oi;

  private Command m_autonomousCommand;
  private SendableChooser<Command> m_chooser;
  private Command autoOne, autoTwo, autoThree;

  private SendableChooser<Integer> m_driveChooser;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  public void robotInit() {
    logger = new Logger(name, year);

    RobotMap.init();

    // Initialize Subsystems
    drive = new DriveTrain(RobotMap.LEFT_MOTOR_CHANNEL, RobotMap.RIGHT_MOTOR_CHANNEL, RobotMap.LEFT_DRIVE_ENCODER_CHANNELS, RobotMap.RIGHT_DRIVE_ENCODER_CHANNELS);
    shooter = new ShooterIntake(
      new int[] {RobotMap.LEFT_SHOOTER_CHANNEL, RobotMap.RIGHT_SHOOTER_CHANNEL}, 
      new int[] {RobotMap.LEFT_INTAKE_CHANNEL, RobotMap.RIGHT_INTAKE_CHANNEL}, 
      new int[] {RobotMap.LEFT_SHOOTER_SOLENOID_CHANNEL, RobotMap.RIGHT_SHOOTER_SOLENOID_CHANNEL}
    );
    vision = new PhotonVision();
    lift = new Lift(RobotMap.LEFT_LIFT_CHANNEL, RobotMap.RIGHT_LIFT_CHANNEL);

    m_oi = new OI();

    autoOne = new AutoOne(drive);
    autoTwo = new AutoTwo(vision, drive, shooter);
    autoThree = new AutoThree(vision, drive, shooter);
    m_chooser = new SendableChooser<Command>();

    /* 
     * One --> Drive Back
     * Two --> Drive Back + Speaker Shoot 
     * Three --> Drive Back + Amp Shoot
     */
    m_chooser.setDefaultOption("Drive Back", autoOne);
    m_chooser.addOption("Drive Back & Speaker Score", autoTwo);
    m_chooser.addOption("Drive Back & Auto Score", autoThree);
    SmartDashboard.putData("Auto mode", m_chooser);

    m_driveChooser = new SendableChooser<Integer>();
    m_driveChooser.setDefaultOption("Tank Drive", 1);
    m_driveChooser.addOption("Arcade Drive", 0);
    SmartDashboard.putData("Drive Mode", m_driveChooser);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode. You
   * can use it to reset any subsystem information you want to clear when the
   * robot is disabled.
   */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString code to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional commands to the
   * chooser code above (like the commented example) or additional comparisons to
   * the switch structure below with additional strings & commands.
   */
  @Override
  public void autonomousInit() {
    System.out.println(SmartDashboard.getKeys());
    m_autonomousCommand = m_chooser.getSelected();
    
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  // comment
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // 0: arcade, 1: tank
    int m_driveMode = m_driveChooser.getSelected();

    //Sets the default command of drive
    drive.setDriveMode(m_driveMode);
    drive.setDefaultCommand(drive.getDefaultCommand());
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
