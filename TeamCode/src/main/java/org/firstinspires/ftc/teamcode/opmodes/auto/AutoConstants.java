package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AutoConstants {
    public static boolean AUTO_AIM = true;
    // 0 = use Limelight distance and the shot table. Above 0 = always shoot at this RPM
    public static double AUTO_SHOT_RPM = 0;
    public static double AUTO_RPM_OFFSET = 50;
    public static double SECOND_GROUP_EXTRA_RPM = -20;
    public static long GROUP2_STEADY_MS = 1000;
    public static long GROUP2_MAX_WAIT_MS = 4000;
    public static long SHOOT_TIME = 5000;
    public static long FIRST_FEED_TIME_MS = 500;
    public static long FEED_PAUSE_TIME_MS = 1000;
    public static long SECOND_FEED_TIME_MS = 500;
    public static long SPIN_AFTER_FEED_MS = 1000;
    public static long AIM_SETTLE_MS = 300;
    public static double AIM_SETTLE_MAX_DEG_PER_SEC = 50;
    public static long TARGET_WAIT_TIME = 500;
    public static long TARGET_SEARCH_TIME = 8000;
    public static double TARGET_SEARCH_TURN_POWER = -0.5;
}
