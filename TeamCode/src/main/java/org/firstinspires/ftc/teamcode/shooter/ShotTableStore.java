package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public final class ShotTableStore {
    private static final String FILE_NAME = "biobuzz_shot_table.txt";

    private ShotTableStore() {}

    private static File file() {
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    public static String path() {
        return file().getAbsolutePath();
    }

    public static boolean save(double[] distances, double[] rpms) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < Math.min(distances.length, rpms.length); i++) {
            sb.append(String.format(Locale.US, "%.2f,%.0f\n", distances[i], rpms[i]));
        }
        try {
            ReadWriteFile.writeFile(file(), sb.toString());
            return true;
        } catch (Exception e) {
            return false;
        }
    }

    public static int load() {
        try {
            File f = file();
            if (!f.exists()) return 0;
            List<double[]> points = new ArrayList<>();
            for (String line : ReadWriteFile.readFile(f).split("\n")) {
                String[] parts = line.trim().split(",");
                if (parts.length != 2) continue;
                points.add(new double[]{Double.parseDouble(parts[0]), Double.parseDouble(parts[1])});
            }
            if (points.isEmpty()) return 0;
            double[] d = new double[points.size()];
            double[] r = new double[points.size()];
            for (int i = 0; i < points.size(); i++) {
                d[i] = points.get(i)[0];
                r[i] = points.get(i)[1];
            }
            ShooterConstants.SHOT_DISTANCES_IN = d;
            ShooterConstants.SHOT_RPMS = r;
            return points.size();
        } catch (Exception e) {
            return 0;
        }
    }
}
