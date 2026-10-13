package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionConstants {
    public static int RED_PIPELINE = 0;    // ID filter: 30-37
    public static int BLUE_PIPELINE = 1;   // ID filter: 38-45

    public static double CAMERA_HEIGHT_IN = 10.0;              // floor to the center of the Limelight lens
    public static double CAMERA_PITCH_DEG = 25.0;              // how far the Limelight is tilted up from level
    public static double CAMERA_FORWARD_OF_SHOOTER_IN = 0.0;   // + if the lens is in front of where the ball leaves the shooter

    public static double UP_CELL_MIN_TAG_HEIGHT_IN = 26.0;

    public static double AIM_POINT_CLOSER_THAN_TAGS_IN = 6.0;

    public static double SMOOTHING = 0.3;          // 0-1: how much each new reading counts (lower = steadier, slower)
    public static long MAX_RESULT_AGE_MS = 100;    // ignore Limelight results older than this
}
