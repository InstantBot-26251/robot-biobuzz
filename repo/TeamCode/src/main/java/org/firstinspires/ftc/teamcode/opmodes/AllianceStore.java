package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public final class AllianceStore {
    private static final String FILE_NAME = "biobuzz_alliance.txt";

    private AllianceStore() {}

    private static File file() {
        return AppUtil.getInstance().getSettingsFile(FILE_NAME);
    }

    public static void save(Alliance alliance) {
        Alliance.setCurrent(alliance);
        try {
            ReadWriteFile.writeFile(file(), alliance.name());
        } catch (Exception ignored) {
        }
    }

    public static Alliance load() {
        try {
            File f = file();
            if (f.exists()) {
                Alliance.setCurrent(Alliance.valueOf(ReadWriteFile.readFile(f).trim()));
            }
        } catch (Exception ignored) {
        }
        return Alliance.getCurrent();
    }
}
