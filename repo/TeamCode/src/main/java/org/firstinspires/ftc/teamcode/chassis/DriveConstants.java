package org.firstinspires.ftc.teamcode.chassis;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class DriveConstants {
    public static double RED_DRIVER_FORWARD_DEG = 90;
    public static double BLUE_DRIVER_FORWARD_DEG = 270;

    public static double AIM_P = 1.6;
    public static double AIM_D = 0.08;
    public static double AIM_KS = 0.04;
    public static double AIM_MAX_TURN = 0.8;
    public static double AIM_TOLERANCE_DEG = 2.0;

    public static long AIM_TIMEOUT_MS = 1500;
    public static long AIM_SETTLE_MS = 120;

    public static double AIM_FEED_TOLERANCE_DEG = 3.0;

    public static double SLOW_MODE_DRIVE_SCALE = 0.35;
    public static double SLOW_MODE_TURN_SCALE = 0.45;

    public static boolean RUMBLE_FEEDBACK = true;

    public static boolean RUNAWAY_GUARD = true;
    public static long RUNAWAY_GUARD_MS = 1000;
    public static double RUNAWAY_MIN_SPEED = 2.0;
    public static double RUNAWAY_MIN_POWER = 0.3;

    public static long PATH_TIMEOUT_MS = 6000;

    public static boolean DRAW_FIELD = true;
}
