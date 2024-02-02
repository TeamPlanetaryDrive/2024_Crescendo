package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
    //Photonvision Constants
    public static final String kPHOTONVISION_CAMERA_NAME = "photonvision";

    public static final double kCAMERA_HEIGHT_METERS = -1;
    public static final double kCAMERA_PITCH_RADIANS = -1;

    public static final int kAPRIL_TAG_ID_AMP_BLUE = 6;
    public static final int kAPRIL_TAG_ID_AMP_RED = 5;
    public static final double kSHOOTING_DISTANCE_TO_AMP_FEET = -1;
    public static final double kAMP_HEIGHT_METERS = Units.inchesToMeters(48.125);

    public static final int kAPRIL_TAG_ID_SPEAKER_BLUE = 7;
    public static final int kAPRIL_TAG_ID_SPEAKER_RED = 4;
    public static final double kSHOOTING_DISTANCE_TO_SPEAKER_FEET = -1;
    public static final double kSPEAKER_HEIGHT_METERS = Units.inchesToMeters(51.875);

    //Shooter constants
    public static final double kSHOOTING_SPEED = 1;
    public static final double kINTAKE_SPEED = .65;

    //Drive Constants
    public static final double kMAX_ACCELERATION_METERS_PER_SECOND = 3.5;
    public static final double kLEFT_ENCODER_FEET_PER_PULSE = 4./256.;
    public static final double kRIGHT_ENCODER_FEET_PER_PULSE_FEET = 4./256.;

    //Lift Constants
    public static final double kLIFT_SPEED = 0.6;
}
