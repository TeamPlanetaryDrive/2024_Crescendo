package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    //Photonvision Constants
    public static final String kPHOTONVISION_CAMERA_NAME = "photonvision";

    public static final double kCAMERA_HEIGHT_METERS = -1;
    public static final double kCAMERA_PITCH_RADIANS = -1;

    public static final int kAPRIL_TAG_ID_SPEAKER_BLUE = 7;
    public static final int kAPRIL_TAG_ID_SPEAKER_RED = 4;
    public static final double kSHOOTING_DISTANCE_TO_SPEAKER_METERS = Units.feetToMeters(3.25);
    public static final double kSPEAKER_HEIGHT_METERS = Units.inchesToMeters(51.875);

    //Shooter constants
    public static final double kSHOOTING_SPEED = .8;
    public static final double kINTAKE_SPEED = .65;

    //Drive Constants
    public static final double kAUTO_MAX_SPEED_METERS_PER_SECOND = 5;
    public static final double kAUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 1;
    public static final double kMAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2;
    public static final double kLEFT_ENCODER_METERS_PER_PULSE = .47877872/90.;
    public static final double kRIGHT_ENCODER_METERS_PER_PULSE = .47877872/90.;

    public static final double ksVOLTS = 0;
    public static final double kvVOLT_SECONDS_PER_METER = 0;
    public static final double kaVOLT_SECONDS_SQUARED_PER_METER = 0;

    public static final double kP_DRIVE_VELOCITY = 0;

    public static final double kTrackWidthMeters = Units.inchesToMeters(21);
    public static final DifferentialDriveKinematics kDRIVE_KINEMATICS = new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final double kRAMSETE_B = 2;
    public static final double kRAMSETE_ZETA = .7;

    //Lift Constants
    public static final double kLIFT_SPEED = 0.6;
}
