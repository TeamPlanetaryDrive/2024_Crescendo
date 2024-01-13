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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  public static String name = "ROB";
  public static Integer year = 2024;

  // Declare bot utilities
  public static Logger logger;

  // Declare Subsystems as Instance Variables
  public static DriveTrain drive;
  public static Shooter shooter;

  public static OI m_oi;

  private Command m_autonomousCommand;
  private SendableChooser<Command> m_chooser;

  private SendableChooser<Integer> m_driveChooser;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */

  public void robotInit() {
    logger = new Logger(name, year);

    RobotMap.init();

    //Initialize Subsystems
    drive = new DriveTrain(RobotMap.LEFT_MOTOR_CHANNEL, RobotMap.RIGHT_MOTOR_CHANNEL);

    m_oi = new OI();

    //Example on how we would do this
    m_chooser = new SendableChooser<Command>();
    m_chooser.setDefaultOption("auto1", new ParallelCommandGroup());
    // m_chooser.addOption("auto2", new auto2());
    // m_chooser.addOption("auto3", new auto3());
    SmartDashboard.putData("Auto mode", m_chooser);

    m_driveChooser = new SendableChooser<Integer>();
    m_driveChooser.setDefaultOption("ArcadeDrive", 0);
    m_driveChooser.addOption("Tank Drive", 1);
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
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.updateValues();
    CommandScheduler.getInstance().run();
  }

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
  }

  /**
   * This function is called periodically during autonomous.
   */
  //comment
  @Override
  public void autonomousPeriodic() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    CommandScheduler.getInstance().run();
    m_autonomousCommand = null;
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    //0: arcade, 1: tank
    int m_driveMode = m_driveChooser.getSelected();
    drive.setDriveMode(m_driveMode);
    System.out.println(m_driveMode);
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    //}
    
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
