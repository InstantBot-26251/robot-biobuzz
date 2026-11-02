package org.firstinspires.ftc.teamcode.util;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.pedropathing.math.Pose;

public final class PanelsDrawing {
    private static final double ROBOT_RADIUS = 9;
    private static FieldManager field;

    private PanelsDrawing() {}

    public static void init() {
        field = PanelsField.INSTANCE.getField();
        field.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    public static void drawRobot(Pose pose, String color) {
        if (field == null || pose == null) return;
        field.setStyle(PanelsField.INSTANCE.getTRANSPARENT(), color, 0.75);
        field.moveCursor(pose.x(), pose.y());
        field.circle(ROBOT_RADIUS);
        field.moveCursor(pose.x(), pose.y());
        field.line(pose.x() + ROBOT_RADIUS * Math.cos(pose.heading()),
                pose.y() + ROBOT_RADIUS * Math.sin(pose.heading()));
    }

    public static void drawTarget(double x, double y, boolean live) {
        if (field == null) return;
        String color = live ? "#4CAF50" : "#FFC107";
        field.setStyle(color, color, 0.5);
        field.moveCursor(x, y);
        field.circle(2.5);
    }

    public static void update() {
        if (field != null) field.update();
    }
}
