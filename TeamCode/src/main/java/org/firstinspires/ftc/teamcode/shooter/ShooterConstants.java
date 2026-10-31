package org.firstinspires.ftc.teamcode.shooter;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterConstants {
    public static double[] SHOT_DISTANCES_IN = {};
    public static double[] SHOT_RPMS = {};

    public static double DEFAULT_RPM = 4500;

    public static double MAX_EXTRAPOLATE_IN = 8;

    public static double MIN_RPM = 1500;
    public static double MAX_RPM = 5800;

    public static double MAX_ACHIEVABLE_RPM = 6000;
    public static double TICKS_PER_REVOLUTION = 28;

    public static double SHOOTER_KV = 1.0 / 5800.0;
    public static double SHOOTER_KS = 0.02;
    public static double SHOOTER_P = 0.0006;
    public static double SHOOTER_I = 0.0;
    public static double SHOOTER_I_MAX_POWER = 0.1;
    public static double SPINUP_BOOST_RPM = 500;
    public static double NOMINAL_VOLTAGE = 12.0;

    public static double SHOOTER_VELOCITY_TOLERANCE = 100;
    public static long READY_STABLE_MS = 60;
    public static boolean GATE_FEED_ON_SPEED = true;
    public static long FEED_GATE_TIMEOUT_MS = 1500;

    public static VelocitySource VELOCITY_SOURCE = VelocitySource.AVERAGE;
    public static double VELOCITY_FILTER = 0.3;

    public static double MAX_SLOWDOWN_POWER = 0.3;

    public static double SERVO_POWER = 0.75;
    public static double MANUAL_TRIM_STEP_RPM = 50;

    public enum VelocitySource { RIGHT, LEFT, AVERAGE }

    public static double toTicksPerSec(double rpm) {
        return (rpm / 60) * TICKS_PER_REVOLUTION;
    }

    public static double toRpm(double ticksPerSec) {
        return ticksPerSec / TICKS_PER_REVOLUTION * 60;
    }
}
