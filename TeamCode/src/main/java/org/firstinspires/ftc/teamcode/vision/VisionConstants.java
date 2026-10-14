package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionConstants {
    public static int RED_PIPELINE = 0;
    public static int BLUE_PIPELINE = 1;

    public static double CAMERA_HEIGHT_IN = 10.0;              // floor to the center of the limelight lens
    public static double CAMERA_PITCH_DEG = 50.0;              // how far the Limelight is tilted up from level
    public static double CAMERA_FORWARD_OF_SHOOTER_IN = 0.0;

    public static double UP_CELL_MIN_TAG_HEIGHT_IN = 26.0;

    public static double AIM_POINT_CLOSER_THAN_TAGS_IN = 6.0;

    public static double SMOOTHING = 0.3;          // 0-1: how much each new reading counts
    public static long MAX_RESULT_AGE_MS = 20;    // ignore Limelight results older than this
}
