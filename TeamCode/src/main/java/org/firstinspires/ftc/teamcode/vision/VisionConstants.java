package org.firstinspires.ftc.teamcode.vision;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class VisionConstants {
    public static double CAMERA_HEIGHT_IN = 10.0;
    public static double CAMERA_PITCH_DEG = 31.0;
    public static double CAMERA_FORWARD_IN = 0.0;
    public static double CAMERA_LEFT_IN = 0.0;

    public static String LIMELIGHT_NAME = "limelight";
    public static int APRILTAG_PIPELINE = 2;
    public static double CAMERA_YAW_DEG = 0.0;

    public static double UP_CELL_TAG_HEIGHT_IN = 49.9;
    public static double DOWN_CELL_TAG_HEIGHT_IN = 35.6;
    public static boolean USE_TAG_RANGE = true;

    public static int FILTER_SAMPLES = 5;
    public static long FILTER_WINDOW_MS = 600;
    public static long TARGET_MEMORY_MS = 4000;
    public static long MAX_FRAME_AGE_MS = 150;

    public static boolean USE_FIELD_MAP = false;
    public static double HIVE_CENTER_X_IN = 72;
    public static double RED_HIVE_Y_IN = 59.25;
    public static double BLUE_HIVE_Y_IN = 84.75;
    public static double UP_CELL_TAG_OFFSET_IN = 13.04;
    public static double DOWN_CELL_TAG_OFFSET_IN = 11.68;
    public static int ASSUMED_UP_CELL_AT_START = 0;
    public static double MAP_AGREE_IN = 12;
    public static double MAP_REJECT_IN = 20;
    public static int MAP_REJECT_COUNT = 5;
}
