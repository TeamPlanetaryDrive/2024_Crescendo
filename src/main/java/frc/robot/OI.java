/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.commands.lift.*;
// import frc.robot.commands.shooter.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.lift.LiftExtend;
import frc.robot.commands.lift.LiftRetract;
import frc.robot.commands.shooter.Intake;
import frc.robot.commands.shooter.IntakeBackwards;
import frc.robot.commands.shooter.IntakeFromShooter;
import frc.robot.commands.shooter.Shoot;
import frc.robot.commands.shooter.ShootSlow;
/**
 * This class is what binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  /**
   * Maps each XBox controller button onto a command if needed
   */
  public OI() {
    RobotMap.aButton.whileTrue(new IntakeFromShooter(Robot.shooter));   
    RobotMap.bButton.whileTrue(new Shoot(Robot.shooter));
    RobotMap.xButton.whileTrue(new Intake(Robot.shooter));
    RobotMap.yButton.whileTrue(new ShootSlow(Robot.shooter));
    RobotMap.startButton.whileTrue(new IntakeBackwards(Robot.shooter));
    // // RobotMap.backButton.onTrue();
    RobotMap.leftBumper.whileTrue(new LiftRetract(Robot.lift));
    RobotMap.rightBumper.whileTrue(new LiftExtend(Robot.lift));
    
    // RobotMap.leftStickButton.whileTrue();
    // RobotMap.rightStickButton.whileTrue();
    
    /*
    Goals for Xbox Controller Button Pressing Mapping
    -------------------------------------------------
    D-Pad: 
    Left Stick: 
    Right Stick: 
    Left Trigger: 
    Right Trigger: 
    Left Bumper: 
    Right Bumper: 
    X: 
    Y: 
    A: 
    B: 
    */
  }
}
