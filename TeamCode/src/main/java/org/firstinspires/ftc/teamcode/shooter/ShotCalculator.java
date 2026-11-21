package org.firstinspires.ftc.teamcode.shooter;

import java.util.Arrays;

public final class ShotCalculator {
    private static final double MERGE_POINTS_WITHIN_IN = 3.0;

    private ShotCalculator() {}

    public static double rpmForDistance(double distanceIn, double[] distances, double[] rpms,
                                        double maxExtrapolateIn, double minRpm, double maxRpm) {
        if (distances == null || rpms == null || Double.isNaN(distanceIn)) return Double.NaN;

        int n = Math.min(distances.length, rpms.length);
        double[][] points = new double[n][];
        int count = 0;
        for (int i = 0; i < n; i++) {
            if (distances[i] > 0 && rpms[i] > 0) points[count++] = new double[]{distances[i], rpms[i]};
        }
        if (count == 0) return Double.NaN;
        points = Arrays.copyOf(points, count);
        Arrays.sort(points, (a, b) -> Double.compare(a[0], b[0]));
        points = mergeClosePoints(points);
        count = points.length;

        double rpm;
        if (count == 1) {
            rpm = points[0][1];
        } else if (distanceIn <= points[0][0]) {
            double d = Math.max(distanceIn, points[0][0] - Math.max(0, maxExtrapolateIn));
            rpm = line(points[0], points[1], d);
        } else if (distanceIn >= points[count - 1][0]) {
            double d = Math.min(distanceIn, points[count - 1][0] + Math.max(0, maxExtrapolateIn));
            rpm = line(points[count - 2], points[count - 1], d);
        } else {
            rpm = smooth(points, distanceIn);
        }
        return Math.max(minRpm, Math.min(maxRpm, rpm));
    }

    private static double smooth(double[][] p, double d) {
        int k = 0;
        while (k < p.length - 2 && d > p[k + 1][0]) k++;
        double h = p[k + 1][0] - p[k][0];
        double t = (d - p[k][0]) / h;
        double m0 = slopeAt(p, k), m1 = slopeAt(p, k + 1);
        return (1 + 2 * t) * (1 - t) * (1 - t) * p[k][1]
                + t * (1 - t) * (1 - t) * h * m0
                + t * t * (3 - 2 * t) * p[k + 1][1]
                + t * t * (t - 1) * h * m1;
    }

    private static double secant(double[][] p, int i) {
        return (p[i + 1][1] - p[i][1]) / (p[i + 1][0] - p[i][0]);
    }

    private static double slopeAt(double[][] p, int i) {
        int n = p.length;
        if (n == 2) return secant(p, 0);
        if (i == 0) return endSlope(p[1][0] - p[0][0], p[2][0] - p[1][0], secant(p, 0), secant(p, 1));
        if (i == n - 1) return endSlope(p[n - 1][0] - p[n - 2][0], p[n - 2][0] - p[n - 3][0], secant(p, n - 2), secant(p, n - 3));
        double before = secant(p, i - 1), after = secant(p, i);
        if (before * after <= 0) return 0;
        double hBefore = p[i][0] - p[i - 1][0], hAfter = p[i + 1][0] - p[i][0];
        double w1 = 2 * hAfter + hBefore, w2 = hAfter + 2 * hBefore;
        return (w1 + w2) / (w1 / before + w2 / after);
    }

    private static double endSlope(double h0, double h1, double s0, double s1) {
        double m = ((2 * h0 + h1) * s0 - h0 * s1) / (h0 + h1);
        if (Math.signum(m) != Math.signum(s0)) return 0;
        if (Math.signum(s0) != Math.signum(s1) && Math.abs(m) > 3 * Math.abs(s0)) return 3 * s0;
        return m;
    }

    private static double[][] mergeClosePoints(double[][] sorted) {
        double[][] merged = new double[sorted.length][];
        int count = 0;
        int i = 0;
        while (i < sorted.length) {
            double first = sorted[i][0];
            double sumDistance = 0, sumRpm = 0;
            int n = 0;
            while (i < sorted.length && sorted[i][0] - first <= MERGE_POINTS_WITHIN_IN) {
                sumDistance += sorted[i][0];
                sumRpm += sorted[i][1];
                n++;
                i++;
            }
            merged[count++] = new double[]{sumDistance / n, sumRpm / n};
        }
        return Arrays.copyOf(merged, count);
    }

    private static double line(double[] a, double[] b, double d) {
        double span = b[0] - a[0];
        if (Math.abs(span) < 1e-9) return 0.5 * (a[1] + b[1]);
        return a[1] + (d - a[0]) / span * (b[1] - a[1]);
    }
}
