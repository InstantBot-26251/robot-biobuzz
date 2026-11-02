package org.firstinspires.ftc.teamcode.vision;

public final class TargetMath {
    private TargetMath() {}

    public static double[] targetInRobotFrame(double txDeg, double tyDeg, double targetHeightIn,
                                              double camHeightIn, double camPitchDeg, double camYawDeg,
                                              double camForwardIn, double camLeftIn) {
        double right = Math.tan(Math.toRadians(txDeg));
        double up = Math.tan(Math.toRadians(tyDeg));
        double pitch = Math.toRadians(camPitchDeg);

        double forward = Math.cos(pitch) - up * Math.sin(pitch);
        double vertical = Math.sin(pitch) + up * Math.cos(pitch);
        if (Math.abs(vertical) < 1e-3) return null;

        double scale = (targetHeightIn - camHeightIn) / vertical;
        if (scale <= 0 || Double.isNaN(scale) || Double.isInfinite(scale)) return null;

        double cx = scale * forward;
        double cy = -scale * right;

        double yaw = Math.toRadians(camYawDeg);
        double x = camForwardIn + cx * Math.cos(yaw) - cy * Math.sin(yaw);
        double y = camLeftIn + cx * Math.sin(yaw) + cy * Math.cos(yaw);
        return new double[]{x, y};
    }

    public static double heightFromRange(double txDeg, double tyDeg, double rangeIn,
                                         double camHeightIn, double camPitchDeg) {
        double right = Math.tan(Math.toRadians(txDeg));
        double up = Math.tan(Math.toRadians(tyDeg));
        double pitch = Math.toRadians(camPitchDeg);
        double norm = Math.sqrt(right * right + up * up + 1.0);
        double vertical = (Math.sin(pitch) + up * Math.cos(pitch)) / norm;
        return camHeightIn + rangeIn * vertical;
    }

    public static double elevation(double tyDeg, double camPitchDeg) {
        double up = Math.tan(Math.toRadians(tyDeg));
        double pitch = Math.toRadians(camPitchDeg);
        return Math.atan2(Math.sin(pitch) + up * Math.cos(pitch), Math.cos(pitch) - up * Math.sin(pitch));
    }

    public static double[] robotToField(double[] robotPoint, double robotX, double robotY, double robotHeading) {
        double c = Math.cos(robotHeading);
        double s = Math.sin(robotHeading);
        return new double[]{
                robotX + robotPoint[0] * c - robotPoint[1] * s,
                robotY + robotPoint[0] * s + robotPoint[1] * c
        };
    }

    public static double median(double[] values, int count) {
        double[] copy = new double[count];
        System.arraycopy(values, 0, copy, 0, count);
        java.util.Arrays.sort(copy);
        if (count % 2 == 1) return copy[count / 2];
        return 0.5 * (copy[count / 2 - 1] + copy[count / 2]);
    }

    public static double normalizeAngle(double radians) {
        double a = radians % (2 * Math.PI);
        if (a > Math.PI) a -= 2 * Math.PI;
        if (a <= -Math.PI) a += 2 * Math.PI;
        return a;
    }
}
