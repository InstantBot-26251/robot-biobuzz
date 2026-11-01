package org.firstinspires.ftc.teamcode.shooter;

import java.util.Arrays;

public final class ShotCalculator {
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
            rpm = points[count - 1][1];
            for (int i = 0; i < count - 1; i++) {
                if (distanceIn <= points[i + 1][0]) {
                    rpm = line(points[i], points[i + 1], distanceIn);
                    break;
                }
            }
        }
        return Math.max(minRpm, Math.min(maxRpm, rpm));
    }

    private static double line(double[] a, double[] b, double d) {
        double span = b[0] - a[0];
        if (Math.abs(span) < 1e-9) return 0.5 * (a[1] + b[1]);
        return a[1] + (d - a[0]) / span * (b[1] - a[1]);
    }
}
