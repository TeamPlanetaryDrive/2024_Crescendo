/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
        public static final double PERIODIC_UPDATE_PERIOD = 0.020; // Periodic update period (s)

        // Power Channels
        public static final int DRIVE_POWER_LEFT_FRONT = 837148, DRIVE_POWER_RIGHT_FRONT = 378149,
                        DRIVE_POWER_LEFT_REAR = 893149, DRIVE_POWER_RIGHT_REAR = 81340723, CAMERA_ONE_POWER = 0,
                        CAMERA_TWO_POWER = 1, VRM_POWER = 38578942;

        // Camera Resolution Dimensions
        public static final int CAM_WID = 640, CAM_HEI = 480;

        // Drive system PID Parameters
        public static final double DRIVE_PID_POSITION_KP = 2.00, // 2.00
                        DRIVE_PID_POSITION_KI = 0.01, // 0.01
                        DRIVE_PID_POSITION_KD = 1.00, // 1.00
                        DRIVE_PID_ANGLE_KP = 0.02, DRIVE_PID_ANGLE_KI = 0.001, DRIVE_PID_ANGLE_KD = 0.0;

        //CHANNELS!!!
        
        // Digital (0-9, 10-25)
        // 2019 bot
        //public static int LEFT_MOTOR_CHANNEL = 0, RIGHT_MOTOR_CHANNEL = 1;
        // 2022 bot
        public static int LEFT_MOTOR_CHANNEL = 1, RIGHT_MOTOR_CHANNEL = 0;
        public static int LEFT_SHOOTER_CHANNEL = 2, RIGHT_SHOOTER_CHANNEL = 3;
        public static int LEFT_INTAKE_CHANNEL = 4, RIGHT_INTAKE_CHANNEL = 5;
        public static int LEFT_LIFT_CHANNEL = 6, RIGHT_LIFT_CHANNEL = 7;

        public static int LEFT_SHOOTER_SOLENOID_CHANNEL = 0, RIGHT_SHOOTER_SOLENOID_CHANNEL = 1;

        public static double PI_CAMERA_HEIGHT = 5, PI_CAMERA_PITCH = 0;
        
        //Controller
        public static JoystickButton aButton, bButton, xButton, yButton, backButton, startButton, leftBumper, rightBumper, leftStickButton, rightStickButton;
        public static final int XBOX_PORT = 0;
        public static final XboxController XController = new XboxController(XBOX_PORT);

        public static void init() {
                //map each button to a JoystickButton
                aButton = new JoystickButton(XController, 1);
                bButton = new JoystickButton(XController, 2);
                xButton = new JoystickButton(XController, 3);
                yButton = new JoystickButton(XController, 4);
                leftBumper = new JoystickButton(XController, 5);
                rightBumper = new JoystickButton(XController, 6);
                backButton = new JoystickButton(XController, 7);
                startButton = new JoystickButton(XController, 8);
                leftStickButton = new JoystickButton(XController, 9);
                rightStickButton = new JoystickButton(XController, 10);
        }
}