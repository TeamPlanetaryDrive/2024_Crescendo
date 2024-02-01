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
        
        public static int[] LEFT_DRIVE_ENCODER_CHANNELS = {0, 1}, RIGHT_DRIVE_ENCODER_CHANNELS = {2, 3};

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