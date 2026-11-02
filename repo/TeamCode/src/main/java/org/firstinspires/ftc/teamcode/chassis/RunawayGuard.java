package org.firstinspires.ftc.teamcode.chassis;

import static org.firstinspires.ftc.teamcode.chassis.DriveConstants.*;

public class RunawayGuard {
    private double lastX, lastY;
    private long lastNanos;
    private boolean hasLast = false;
    private long badSinceNanos = -1;
    private double badStartSpeed = 0;

    public void reset() {
        hasLast = false;
        badSinceNanos = -1;
    }

    public String update(long nowNanos, double x, double y, double heading,
                         boolean driving, double cmdForward, double cmdLeft) {
        if (!RUNAWAY_GUARD || !driving || !hasLast) {
            badSinceNanos = -1;
            lastX = x;
            lastY = y;
            lastNanos = nowNanos;
            hasLast = true;
            return null;
        }
        double dt = (nowNanos - lastNanos) / 1e9;
        if (dt < 0.02) return null;
        double dx = x - lastX;
        double dy = y - lastY;
        double vForward = (Math.cos(heading) * dx + Math.sin(heading) * dy) / dt;
        double vLeft = (-Math.sin(heading) * dx + Math.cos(heading) * dy) / dt;
        lastX = x;
        lastY = y;
        lastNanos = nowNanos;

        double cmd = Math.hypot(cmdForward, cmdLeft);
        double speed = Math.hypot(vForward, vLeft);
        if (cmd < RUNAWAY_MIN_POWER) {
            badSinceNanos = -1;
            return null;
        }
        double agreement = speed < 1e-6 ? 0 : (vForward * cmdForward + vLeft * cmdLeft) / (speed * cmd);
        boolean bad = speed < RUNAWAY_MIN_SPEED || agreement < 0;
        if (!bad) {
            badSinceNanos = -1;
            return null;
        }
        if (badSinceNanos < 0) {
            badSinceNanos = nowNanos;
            badStartSpeed = speed;
        }
        if ((nowNanos - badSinceNanos) / 1e6 > RUNAWAY_GUARD_MS) {
            if (speed < RUNAWAY_MIN_SPEED) {
                return "OTOS sees almost no motion while driving (height / dirty lens / cover film)";
            } else if (speed >= 0.7 * badStartSpeed) {
                return "OTOS motion is the WRONG direction (mount angle: run OTOS Calibration)";
            } else {
                badSinceNanos = -1;
            }
        }
        return null;
    }
}
