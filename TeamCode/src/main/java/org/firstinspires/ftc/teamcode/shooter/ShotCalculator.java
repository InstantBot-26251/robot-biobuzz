package org.firstinspires.ftc.teamcode.shooter;

import static org.firstinspires.ftc.teamcode.vision.VisionConstants.*;

public final class ShotCalculator {
    public static final double GRAVITY_IN_PER_S2 = 386.09;

    private ShotCalculator() {}

    public static double requiredBallSpeed(double distanceIn, double heightGainIn, double launchAngleDeg) {
        double angle = Math.toRadians(launchAngleDeg);
        double cos = Math.cos(angle);
        double rise = distanceIn * Math.tan(angle) - heightGainIn;
        if (distanceIn <= 0 || cos <= 0 || rise <= 0) {
            return Double.NaN;
        }
        return Math.sqrt(GRAVITY_IN_PER_S2 * distanceIn * distanceIn / (2 * cos * cos * rise));
    }

    public static double rpmForDistance(double distanceIn) {
        double heightGain = HIVE_OPENING_HEIGHT_IN - LAUNCH_HEIGHT_IN;
        double needed = requiredBallSpeed(distanceIn, heightGain, LAUNCH_ANGLE_DEG);
        double atCalibration = requiredBallSpeed(CAL_DISTANCE_IN, heightGain, LAUNCH_ANGLE_DEG);
        if (Double.isNaN(needed) || Double.isNaN(atCalibration)) {
            return Double.NaN;
        }
        double trim = 1.0 + LONG_SHOT_TRIM * (distanceIn - CAL_DISTANCE_IN) / CAL_DISTANCE_IN;
        return CAL_RPM * (needed / atCalibration) * trim;
    }

    public static double targetRpm(double distanceIn) {
        double rpm = rpmForDistance(distanceIn);
        if (Double.isNaN(rpm)) {
            return Double.NaN;
        }
        return Math.max(MIN_AUTO_RPM, Math.min(MAX_AUTO_RPM, rpm));
    }
}
