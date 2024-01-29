/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import frc.robot.commands.combinedcommands.*;
import frc.robot.commands.lift.*;
import frc.robot.commands.shooter.*;

/**
 * This class is what binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

  public OI() {
    RobotMap.aButton.onTrue(new AutomaticallyShoot(
      Robot.vision, Robot.drive, Robot.shooter, 3, .1
    ));    
    // RobotMap.bButton.onTrue();
    RobotMap.xButton.onTrue(new Intake(Robot.shooter));
    // RobotMap.yButton.onTrue(); 
    // RobotMap.startButton.onTrue();
    // RobotMap.backButton.onTrue();
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
