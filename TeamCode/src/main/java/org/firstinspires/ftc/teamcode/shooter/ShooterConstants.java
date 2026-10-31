package org.firstinspires.ftc.teamcode.shooter;

public class ShooterConstants {

    public static double MAX_ACHIEVABLE_RPM = 6000;
    public static  double TICKS_PER_REVOLUTION = 28;

    public static double SHOOTER_VELOCITY = 3000; // in RPM
    public static double SHOOTER_VELOCITY_TOLERANCE = 150;

    public static double SHOOTER_P = 1;
    public static double SHOOTER_I = 0;
    public static double SHOOTER_D = 0;
    public static double SHOOTER_F = 0;

    public static double SERVO_POWER = 0.75;
    public static long SERVO_WAIT_TIME = 1;
    public static double SHOOTER_POWER = -0.55; // TODO: tune on robot
    public static long POST_CYCLE_WAIT_TIME = 50;

    public static double toTicksPerSec(double rpm) {
        return (rpm / 60) * TICKS_PER_REVOLUTION;
    }
}