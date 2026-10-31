package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.chassis.DriveConstants;

public enum Alliance {
    RED(30, 37),
    BLUE(38, 45);

    private static Alliance current = RED;

    public final int firstTagId;
    public final int lastTagId;

    Alliance(int firstTagId, int lastTagId) {
        this.firstTagId = firstTagId;
        this.lastTagId = lastTagId;
    }

    public boolean ownsTag(int id) {
        return id >= firstTagId && id <= lastTagId;
    }

    public double driverForwardHeading() {
        return Math.toRadians(this == RED ? DriveConstants.RED_DRIVER_FORWARD_DEG : DriveConstants.BLUE_DRIVER_FORWARD_DEG);
    }

    public static Alliance getCurrent() {
        return current;
    }

    public static void setCurrent(Alliance alliance) {
        current = alliance;
    }
}
