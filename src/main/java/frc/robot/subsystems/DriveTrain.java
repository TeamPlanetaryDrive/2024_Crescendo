/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Victor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;

public class DriveTrain extends SubsystemBase {
  private DifferentialDrive robotDrive;
  private Victor lMotor, rMotor;
  private int driveMode; //0: Arcade, 1: Tank

  public DriveTrain(int leftMotor, int rightMotor) {
    super();
    lMotor = new Victor(leftMotor);
    rMotor = new Victor(rightMotor);
    rMotor.setInverted(true);
    robotDrive = new DifferentialDrive(lMotor, rMotor);
  }

  public void periodic() {
    if(driveMode == 1) {
      double left = -RobotMap.XController.getLeftY();
      double right = -RobotMap.XController.getRightY();
      robotDrive.tankDrive(left, right);
    }
    else {
      double yax = -RobotMap.XController.getLeftY();
      double xax = -RobotMap.XController.getLeftX();
      robotDrive.arcadeDrive(yax, xax);
    }
    
  }

  public void setDriveMode(int mode) {
    driveMode = mode;
  }
}
