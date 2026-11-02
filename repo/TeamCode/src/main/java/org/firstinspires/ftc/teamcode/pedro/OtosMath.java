package org.firstinspires.ftc.teamcode.pedro;

public final class OtosMath {
    private OtosMath() {}

    public static double mountHeading(double pushedX, double pushedY) {
        return -Math.atan2(pushedY, pushedX);
    }

    public static double[] sensorPosition(double driftX, double driftY, double h0, double h1) {
        double[] d = {Math.cos(h1) - Math.cos(h0), Math.sin(h1) - Math.sin(h0)};
        if (abs(d) < 0.5) return null;
        return div(new double[]{driftX, driftY}, d);
    }

    public static double[] conventionRotation(double[] s, double[] c, double driftX, double driftY, double h0, double h1) {
        double[] sNow = sensorPosition(driftX, driftY, h0, h1);
        if (sNow == null || abs(c) < 1e-6) return null;
        double[] gc = {s[0] - sNow[0], s[1] - sNow[1]};
        double[] g = div(gc, c);
        double mag = abs(g);
        if (mag < 1e-6) return null;
        return new double[]{g[0] / mag, g[1] / mag};
    }

    public static double[] solveOffset(double[] s, double[] g) {
        return div(s, g);
    }

    public static double abs(double[] z) {
        return Math.hypot(z[0], z[1]);
    }

    public static double[] div(double[] a, double[] b) {
        double den = b[0] * b[0] + b[1] * b[1];
        return new double[]{(a[0] * b[0] + a[1] * b[1]) / den, (a[1] * b[0] - a[0] * b[1]) / den};
    }

    public static double[] mul(double[] a, double[] b) {
        return new double[]{a[0] * b[0] - a[1] * b[1], a[0] * b[1] + a[1] * b[0]};
    }
}
