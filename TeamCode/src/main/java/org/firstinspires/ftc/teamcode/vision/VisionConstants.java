package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

@Config
public class VisionConstants {
    public static int RED_PIPELINE = 0;
    public static int BLUE_PIPELINE = 1;

    public static double CAMERA_HEIGHT_IN = 10.0;
    public static double CAMERA_PITCH_DEG = 50.0;
    public static double CAMERA_FORWARD_OF_SHOOTER_IN = 0.0;

    public static double UP_CELL_MIN_TAG_HEIGHT_IN = 26.0;

    public static double AIM_POINT_CLOSER_THAN_TAGS_IN = 6.0;

    public static double SMOOTHING = 0.3;
    public static long MAX_RESULT_AGE_MS = 20;

    public static double CAL_DISTANCE_IN = 60.0;
    public static double CAL_POWER = 0.55;

    public static double LONG_SHOT_TRIM = 0.0;

    public static double LAUNCH_ANGLE_DEG = 20.0;
    public static double LAUNCH_HEIGHT_IN = 12.0;
    public static double HIVE_OPENING_HEIGHT_IN = 40.0;

    public static boolean AUTO_SPIN_UP = false;
    public static long TAG_LOST_HOLD_MS = 750;
    public static double MIN_AUTO_POWER = 0.30;
    public static double MAX_AUTO_POWER = 1.00;
}
