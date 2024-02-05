package frc.robot.commands.photonvision;

import java.util.List;
import java.util.NoSuchElementException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveToAmpRamsete {
    private static final Pose2d RED_AMP_POSE = new Pose2d(14.72, 8, new Rotation2d(Math.PI/2));
    private static final Pose2d BLUE_AMP_POSE = new Pose2d(1.86, 8, new Rotation2d(Math.PI/2));

    public static RamseteCommand getCommand() {
        Pose3d currentPose;
        try{
            currentPose = Robot.vision.getPoseEstimator().update().get().estimatedPose;
        } catch(NoSuchElementException E) {
            return null;
        }
        Pose2d currentPose2 = currentPose.toPose2d();

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
    
        Pose2d ampPose;
        
        try {
            if(DriverStation.getAlliance().get() == Alliance.Blue) {
                ampPose = BLUE_AMP_POSE;
            }
            else {
                ampPose = RED_AMP_POSE;
            }
        }
        catch(NoSuchElementException E) {
            ampPose = BLUE_AMP_POSE;
        }

        Trajectory newTraj = TrajectoryGenerator.generateTrajectory(
            currentPose2,
            List.of(
                new Translation2d(currentPose2.getX(), ampPose.getY())
            ),
            ampPose,
            config
        );
        

        RamseteCommand ramseteCommand = new RamseteCommand(
            newTraj,
            Robot.drive::getPose,
            new RamseteController(Constants.kRAMSETE_B, Constants.kRAMSETE_ZETA),
            new SimpleMotorFeedforward(Constants.ksVOLTS, Constants.kvVOLT_SECONDS_PER_METER, Constants.kaVOLT_SECONDS_SQUARED_PER_METER),
            Constants.kDRIVE_KINEMATICS,
            Robot.drive::getWheelSpeeds,
            new PIDController(Constants.kP_DRIVE_VELOCITY, 0, 0),
            new PIDController(Constants.kP_DRIVE_VELOCITY, 0, 0),
            Robot.drive::tankDrive,
            Robot.drive
        );
    
        return ramseteCommand;    
    }
}