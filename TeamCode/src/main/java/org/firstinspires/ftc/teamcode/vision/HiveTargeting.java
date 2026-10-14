package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.teamcode.util.Alliance;

import java.util.List;

public final class HiveTargeting {
    public static final int AUDIENCE_CELL = 0;
    public static final int FAR_CELL = 1;
    public static final int NO_CELL = -1;

    private HiveTargeting() {}

    public static int cellFor(Alliance alliance, int tagId) {
        if (alliance == Alliance.RED) {
            if (tagId >= 34 && tagId <= 37) return AUDIENCE_CELL;
            if (tagId >= 30 && tagId <= 33) return FAR_CELL;
        } else {
            if (tagId >= 38 && tagId <= 41) return AUDIENCE_CELL;
            if (tagId >= 42 && tagId <= 45) return FAR_CELL;
        }
        return NO_CELL;
    }

    public static class TagPosition {
        public final int id;
        public final double forwardIn;
        public final double rightIn;
        public final double heightIn;

        public TagPosition(int id, double forwardIn, double rightIn, double heightIn) {
            this.id = id;
            this.forwardIn = forwardIn;
            this.rightIn = rightIn;
            this.heightIn = heightIn;
        }
    }

    public static TagPosition locate(int id, double rangeIn, double txDeg, double tyDeg,
                                     double cameraHeightIn, double cameraPitchDeg, double cameraForwardOfShooterIn) {
        double tanX = Math.tan(Math.toRadians(txDeg));
        double tanY = Math.tan(Math.toRadians(tyDeg));

        double camForward = rangeIn / Math.sqrt(1 + tanX * tanX + tanY * tanY);
        double camRight = camForward * tanX;
        double camUp = camForward * tanY;

        double pitch = Math.toRadians(cameraPitchDeg);
        double forward = camForward * Math.cos(pitch) - camUp * Math.sin(pitch) + cameraForwardOfShooterIn;
        double height = cameraHeightIn + camForward * Math.sin(pitch) + camUp * Math.cos(pitch);

        return new TagPosition(id, forward, camRight, height);
    }

    public static class Choice {
        public final int cell;
        public final double distanceIn;
        public final double tagHeightIn;

        Choice(int cell, double distanceIn, double tagHeightIn) {
            this.cell = cell;
            this.distanceIn = distanceIn;
            this.tagHeightIn = tagHeightIn;
        }
    }

    public static Choice chooseUpCell(Alliance alliance, List<TagPosition> tags,
                                      double minUpTagHeightIn, double aimPointCloserThanTagsIn) {
        double[] forward = new double[2], right = new double[2], height = new double[2];
        int[] count = new int[2];

        for (TagPosition tag : tags) {
            int cell = cellFor(alliance, tag.id);
            if (cell == NO_CELL) continue;
            forward[cell] += tag.forwardIn;
            right[cell] += tag.rightIn;
            height[cell] += tag.heightIn;
            count[cell]++;
        }
        for (int c = 0; c < 2; c++) {
            if (count[c] > 0) {
                forward[c] /= count[c];
                right[c] /= count[c];
                height[c] /= count[c];
            }
        }

        int up;
        if (count[AUDIENCE_CELL] > 0 && count[FAR_CELL] > 0) {
            up = height[AUDIENCE_CELL] >= height[FAR_CELL] ? AUDIENCE_CELL : FAR_CELL;
        } else if (count[AUDIENCE_CELL] > 0 && height[AUDIENCE_CELL] >= minUpTagHeightIn) {
            up = AUDIENCE_CELL;
        } else if (count[FAR_CELL] > 0 && height[FAR_CELL] >= minUpTagHeightIn) {
            up = FAR_CELL;
        } else {
            return new Choice(NO_CELL, Double.NaN, Double.NaN);
        }

        double distance = Math.hypot(forward[up], right[up]) - aimPointCloserThanTagsIn;
        return new Choice(up, distance, height[up]);
    }
}
