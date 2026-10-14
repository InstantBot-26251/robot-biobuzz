package org.firstinspires.ftc.teamcode.shooter;


import com.acmerobotics.dashboard.config.Config;
@Config
public class ShooterConstants {


    public static double SHOOTER_POWER = 0.55; // used when no HIVE tag is in view

    public static double SERVO_POWER = 0.75; // TODO: tune on robot
    public static long SERVO_WAIT_TIME = 10;
    public static long POST_CYCLE_WAIT_TIME = 50;

    //AUTO AIM
    public static double CAL_DISTANCE_IN = 60.0;   // the distance where CAL_POWER scores
    public static double CAL_POWER = 0.55;         // the power that scores from CAL_DISTANCE_IN

    public static double LONG_SHOT_TRIM = 0.0;

    public static double LAUNCH_ANGLE_DEG = 50.0;        // angle the ball leaves the shooter, from level
    public static double LAUNCH_HEIGHT_IN = 12.0;        // floor to where the ball leaves the shooter
    public static double HIVE_OPENING_HEIGHT_IN = 40.0;  // floor to the middle of the up CELL's opening

    public static boolean AUTO_AIM_ENABLED = true;       // false = always use SHOOTER_POWER
    public static boolean AUTO_SPIN_UP = false;
    public static long TAG_LOST_HOLD_MS = 750;           // keep the last auto power this long after losing the tag
    public static double MIN_AUTO_POWER = 0.30;
    public static double MAX_AUTO_POWER = 1.00;
    public static boolean VOLTAGE_COMPENSATION = true;
    public static double NOMINAL_VOLTAGE = 12.0;
}
