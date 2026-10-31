package org.firstinspires.ftc.teamcode.util;

public enum Alliance {
    RED,
    BLUE;

    private static Alliance current = RED;

    public static Alliance getCurrent() {
        return current;
    }

    public static void setCurrent(Alliance alliance) {
        current = alliance;
    }
}
