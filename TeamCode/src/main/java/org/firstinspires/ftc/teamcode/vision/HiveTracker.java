package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.opmodes.Alliance;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.vision.VisionConstants.*;

public class HiveTracker {
    public static final double[] TAG_LATERAL_OFFSET_IN = {-6.5, -2.75, 2.75, 6.5};
    private static final double LIVE_WINDOW_MS = 250;
    private static final int BUFFER = 32;

    public static class Tag {
        public final int id;
        public final double tx, ty, range;

        public Tag(int id, double tx, double ty, double range) {
            this.id = id;
            this.tx = tx;
            this.ty = ty;
            this.range = range;
        }
    }

    public enum Source { LIVE, MEMORY, MAP, NONE }

    private Alliance alliance = Alliance.RED;
    private boolean fieldLocalized = false;

    private final double[] sampleX = new double[BUFFER];
    private final double[] sampleY = new double[BUFFER];
    private final double[] sampleT = new double[BUFFER];
    private int sampleHead = 0;
    private int sampleCount = 0;

    private double targetX, targetY;
    private double lastSeenMs = -1e12;
    private int currentUpCell = 0;
    private static final double[][] CELL_AXIS = new double[4][];

    private static final int HEIGHT_SAMPLES = 61;
    private static final double MIN_LEARN_SPACING_IN = 9.0;
    private static final double[] UP_HEIGHT_ABOVE_CAMERA = new double[HEIGHT_SAMPLES];
    private static int heightHead = 0;
    private static int heightCount = 0;
    private double lastFrameHeight = Double.NaN;

    private boolean mapConfirmed = false;
    private boolean mapRejected = false;
    private int mapMismatches = 0;
    private double lastMapErrorIn = Double.NaN;

    private int lastAllianceTags = 0;
    private int lastUpTags = 0;
    private boolean lastUsedRange = false;
    private String lastNote = "-";
    private String lastTagHeights = "-";

    public void reset(Alliance alliance, boolean fieldLocalized) {
        this.alliance = alliance;
        this.fieldLocalized = fieldLocalized;
        sampleCount = 0;
        lastSeenMs = -1e12;
        currentUpCell = Math.max(0, Math.min(1, ASSUMED_UP_CELL_AT_START));
        mapConfirmed = false;
        mapRejected = false;
        mapMismatches = 0;
        lastMapErrorIn = Double.NaN;
        lastNote = "-";
    }

    public Source source(double nowMs) {
        if (sampleCount > 0 && nowMs - lastSeenMs < LIVE_WINDOW_MS) return Source.LIVE;
        if (sampleCount > 0 && nowMs - lastSeenMs < TARGET_MEMORY_MS) return Source.MEMORY;
        if (mapUsable()) return Source.MAP;
        return Source.NONE;
    }

    public boolean hasTarget(double nowMs) {
        return source(nowMs) != Source.NONE;
    }

    public boolean isLive(double nowMs) {
        return source(nowMs) == Source.LIVE;
    }

    public double[] target(double nowMs) {
        switch (source(nowMs)) {
            case LIVE:
            case MEMORY:
                return new double[]{targetX, targetY};
            case MAP:
                return mapPoint(alliance, currentUpCell);
            default:
                return null;
        }
    }

    public boolean mapUsable() {
        return USE_FIELD_MAP && fieldLocalized && !mapRejected;
    }

    public boolean isMapConfirmed() {
        return mapConfirmed;
    }

    public boolean isMapRejected() {
        return mapRejected;
    }

    public double getLastMapErrorIn() {
        return lastMapErrorIn;
    }

    public int getCurrentUpCell() {
        return currentUpCell;
    }

    public int getLastAllianceTags() {
        return lastAllianceTags;
    }

    public int getLastUpTags() {
        return lastUpTags;
    }

    public boolean getLastUsedRange() {
        return lastUsedRange;
    }

    public String getLastNote() {
        return lastNote;
    }

    public String getLastTagHeights() {
        return lastTagHeights;
    }

    public static void forgetLearnedAxes() {
        for (int i = 0; i < CELL_AXIS.length; i++) CELL_AXIS[i] = null;
        heightCount = 0;
        heightHead = 0;
    }

    public static double measuredUpHeightAboveCamera() {
        if (heightCount < 3) return Double.NaN;
        return TargetMath.median(UP_HEIGHT_ABOVE_CAMERA, heightCount);
    }

    public double getLastFrameHeight() {
        return lastFrameHeight;
    }

    private static double upTagHeight(double configured) {
        double measured = measuredUpHeightAboveCamera();
        return Double.isNaN(measured) ? configured : CAMERA_HEIGHT_IN + measured;
    }

    private static void rememberUpHeight(double heightAboveCamera) {
        UP_HEIGHT_ABOVE_CAMERA[heightHead] = heightAboveCamera;
        heightHead = (heightHead + 1) % HEIGHT_SAMPLES;
        heightCount = Math.min(heightCount + 1, HEIGHT_SAMPLES);
    }

    private double cellHeightAboveCamera(List<Sighting> ours, int cell) {
        return cellHeightAboveCamera(ours, cell, 0);
    }

    private double cellHeightAboveCamera(List<Sighting> ours, int cell, double minSpacing) {
        List<double[]> perInch = new ArrayList<>();
        List<Double> offsets = new ArrayList<>();
        double lo = Double.POSITIVE_INFINITY, hi = Double.NEGATIVE_INFINITY;
        for (Sighting s : ours) {
            if (s.cell != cell) continue;
            double[] v = TargetMath.perInchOfHeight(s.tx, s.ty, CAMERA_PITCH_DEG, CAMERA_YAW_DEG);
            if (v == null) continue;
            perInch.add(v);
            offsets.add(TAG_LATERAL_OFFSET_IN[s.slot]);
            lo = Math.min(lo, TAG_LATERAL_OFFSET_IN[s.slot]);
            hi = Math.max(hi, TAG_LATERAL_OFFSET_IN[s.slot]);
        }
        if (perInch.size() < 2 || hi - lo < Math.max(minSpacing, 1e-6)) return Double.NaN;
        return TargetMath.heightFromSpacing(perInch, offsets);
    }

    private void learnHeight(List<Sighting> ours, int upCell, double otherCellHeight) {
        double h = cellHeightAboveCamera(ours, upCell, MIN_LEARN_SPACING_IN);
        lastFrameHeight = h;
        if (Double.isNaN(h) || h <= 0) return;
        double gap = UP_CELL_TAG_HEIGHT_IN - DOWN_CELL_TAG_HEIGHT_IN;
        if (!Double.isNaN(otherCellHeight)) {
            if (h > otherCellHeight) rememberUpHeight(h);
            return;
        }
        double measured = measuredUpHeightAboveCamera();
        double expected = Double.isNaN(measured) ? SIMPLE_TAG_HEIGHT_IN - CAMERA_HEIGHT_IN : measured;
        if (Math.abs(h - expected) < gap / 2) rememberUpHeight(h);
    }

    public String cellName(int cell) {
        int first = alliance.firstTagId + cell * 4;
        return "tags " + first + "-" + (first + 3);
    }

    public static double[] mapPoint(Alliance alliance, int cell) {
        boolean lowX = (alliance == Alliance.RED) == (cell == 0);
        double x = HIVE_CENTER_X_IN + (lowX ? -UP_CELL_TAG_OFFSET_IN : UP_CELL_TAG_OFFSET_IN);
        double y = alliance == Alliance.RED ? RED_HIVE_Y_IN : BLUE_HIVE_Y_IN;
        return new double[]{x, y};
    }

    private static class Sighting {
        int cell, slot;
        double tx, ty, height, elevation;
    }

    public void update(List<Tag> tags, Pose capturePose, double nowMs) {
        List<Sighting> ours = new ArrayList<>();
        StringBuilder heights = new StringBuilder();
        for (Tag tag : tags) {
            if (!alliance.ownsTag(tag.id)) continue;
            Sighting s = new Sighting();
            s.cell = (tag.id - alliance.firstTagId) / 4;
            s.slot = (tag.id - alliance.firstTagId) % 4;
            s.tx = tag.tx;
            s.ty = tag.ty;
            s.elevation = TargetMath.elevation(tag.ty, CAMERA_PITCH_DEG);
            double measured = (!Double.isNaN(tag.range) && tag.range > 1.0)
                    ? TargetMath.heightFromRange(tag.tx, tag.ty, tag.range, CAMERA_HEIGHT_IN, CAMERA_PITCH_DEG)
                    : Double.NaN;
            s.height = USE_TAG_RANGE ? measured : Double.NaN;
            heights.append(heights.length() == 0 ? "" : ", ")
                    .append(tag.id).append("=").append(String.format("%.1f", measured));
            ours.add(s);
        }
        lastTagHeights = heights.length() == 0 ? "-" : heights.toString();
        lastAllianceTags = ours.size();
        lastUpTags = 0;
        if (ours.isEmpty()) return;

        if (SIMPLE_TRACKING) {
            int[] count = new int[2];
            for (Sighting s : ours) count[s.cell]++;
            double[] cellHeight = {cellHeightAboveCamera(ours, 0), cellHeightAboveCamera(ours, 1)};
            int cell;
            double learned = measuredUpHeightAboveCamera();
            if (Double.isNaN(learned)) learned = SIMPLE_TAG_HEIGHT_IN - CAMERA_HEIGHT_IN;
            double halfGap = 0.5 * (UP_CELL_TAG_HEIGHT_IN - DOWN_CELL_TAG_HEIGHT_IN);
            if (!Double.isNaN(cellHeight[0]) && !Double.isNaN(cellHeight[1])) {
                cell = cellHeight[0] >= cellHeight[1] ? 0 : 1;
            } else {
                cell = count[0] == count[1] ? currentUpCell : count[0] > count[1] ? 0 : 1;
                if (!Double.isNaN(cellHeight[cell]) && cellHeight[cell] < learned - halfGap) {
                    switchUpCell(1 - cell);
                    lastNote = cellName(cell) + " is the DOWN cell (measured height)";
                    return;
                }
            }
            lastUsedRange = false;
            switchUpCell(cell);
            learnHeight(ours, cell, cellHeight[1 - cell]);
            addFix(projectCell(ours, cell, upTagHeight(SIMPLE_TAG_HEIGHT_IN), Double.NEGATIVE_INFINITY, capturePose),
                    cell, nowMs, " (simple)");
            return;
        }

        double split = 0.5 * (UP_CELL_TAG_HEIGHT_IN + DOWN_CELL_TAG_HEIGHT_IN);
        double[] heightSum = new double[2];
        int[] heightCount = new int[2];
        double[] elevationSum = new double[2];
        int[] seen = new int[2];
        for (Sighting s : ours) {
            if (!Double.isNaN(s.height)) {
                heightSum[s.cell] += s.height;
                heightCount[s.cell]++;
            }
            elevationSum[s.cell] += s.elevation;
            seen[s.cell]++;
        }

        int upCell;
        lastUsedRange = heightCount[0] + heightCount[1] > 0;
        if (lastUsedRange) {
            if (heightCount[0] > 0 && heightCount[1] > 0) {
                upCell = heightSum[0] / heightCount[0] >= heightSum[1] / heightCount[1] ? 0 : 1;
            } else {
                int c = heightCount[0] > 0 ? 0 : 1;
                if (heightSum[c] / heightCount[c] > split) {
                    upCell = c;
                } else if (seen[1 - c] > 0) {
                    upCell = 1 - c;
                } else {
                    switchUpCell(1 - c);
                    lastNote = "only the DOWN cell is visible";
                    return;
                }
            }
        } else if (seen[0] > 0 && seen[1] > 0) {
            upCell = pickUpCellByGeometry(ours, elevationSum, seen);
        } else {
            int c = seen[0] > 0 ? 0 : 1;
            if (c != currentUpCell && nowMs - lastSeenMs < TARGET_MEMORY_MS) {
                lastNote = "1 cell, no 3D: keeping recent fix on " + cellName(currentUpCell);
                return;
            }
            if (mapUsable() && looksDownOnMap(ours, c, capturePose)) {
                switchUpCell(1 - c);
                lastNote = "1 cell, no 3D: map says it's the DOWN cell";
                return;
            }
            upCell = c;
        }
        switchUpCell(upCell);
        learnHeight(ours, upCell, cellHeightAboveCamera(ours, 1 - upCell));
        addFix(projectCell(ours, upCell, upTagHeight(UP_CELL_TAG_HEIGHT_IN), split, capturePose), upCell, nowMs, " UP");
    }

    private List<double[]> projectCell(List<Sighting> ours, int cell, double tagHeight, double minHeight, Pose capturePose) {
        List<double[]> points = new ArrayList<>();
        for (Sighting s : ours) {
            if (s.cell != cell) continue;
            if (!Double.isNaN(s.height) && s.height < minHeight) continue;
            double[] robotPoint = TargetMath.targetInRobotFrame(s.tx, s.ty, tagHeight,
                    CAMERA_HEIGHT_IN, CAMERA_PITCH_DEG, CAMERA_YAW_DEG, CAMERA_FORWARD_IN, CAMERA_LEFT_IN);
            if (robotPoint == null) continue;
            double[] field = TargetMath.robotToField(robotPoint, capturePose.x(), capturePose.y(), capturePose.heading());
            points.add(new double[]{field[0], field[1], TAG_LATERAL_OFFSET_IN[s.slot]});
        }
        return points;
    }

    private void addFix(List<double[]> points, int cell, double nowMs, String label) {
        lastUpTags = points.size();
        if (points.isEmpty()) {
            lastNote = cellName(cell) + label + ": tags seen but unusable";
            return;
        }

        boolean axisKnown = CELL_AXIS[axisIndex(cell)] != null;
        if (points.size() == 1 && !axisKnown && (nowMs - lastSeenMs < TARGET_MEMORY_MS || mapUsable())) {
            lastNote = cellName(cell) + label + " (1 tag, kept previous fix)";
            return;
        }
        double[] center = clusterCenter(points, cell);
        if (!SIMPLE_TRACKING) checkAgainstMap(center, cell);
        lastNote = cellName(cell) + label;

        sampleX[sampleHead] = center[0];
        sampleY[sampleHead] = center[1];
        sampleT[sampleHead] = nowMs;
        sampleHead = (sampleHead + 1) % BUFFER;
        sampleCount = Math.min(sampleCount + 1, BUFFER);

        int limit = Math.max(1, Math.min(FILTER_SAMPLES, BUFFER));
        double[] xs = new double[limit];
        double[] ys = new double[limit];
        int n = 0;
        for (int i = 0; i < sampleCount && n < limit; i++) {
            int idx = (sampleHead - 1 - i + BUFFER) % BUFFER;
            if (nowMs - sampleT[idx] > FILTER_WINDOW_MS) break;
            xs[n] = sampleX[idx];
            ys[n] = sampleY[idx];
            n++;
        }
        targetX = TargetMath.median(xs, n);
        targetY = TargetMath.median(ys, n);
        lastSeenMs = nowMs;
    }

    private int axisIndex(int cell) {
        return alliance.ordinal() * 2 + cell;
    }

    private int pickUpCellByGeometry(List<Sighting> ours, double[] elevationSum, int[] seen) {
        double expected = UP_CELL_TAG_OFFSET_IN + DOWN_CELL_TAG_OFFSET_IN;
        double[] score = new double[2];
        for (int hypothesis = 0; hypothesis < 2; hypothesis++) {
            double[][] mean = new double[2][2];
            int[] count = new int[2];
            for (Sighting s : ours) {
                double height = s.cell == hypothesis ? UP_CELL_TAG_HEIGHT_IN : DOWN_CELL_TAG_HEIGHT_IN;
                double[] p = TargetMath.targetInRobotFrame(s.tx, s.ty, height, CAMERA_HEIGHT_IN,
                        CAMERA_PITCH_DEG, CAMERA_YAW_DEG, CAMERA_FORWARD_IN, CAMERA_LEFT_IN);
                if (p == null) continue;
                mean[s.cell][0] += p[0];
                mean[s.cell][1] += p[1];
                count[s.cell]++;
            }
            if (count[0] == 0 || count[1] == 0) {
                score[hypothesis] = Double.MAX_VALUE;
                continue;
            }
            double sep = Math.hypot(mean[0][0] / count[0] - mean[1][0] / count[1],
                    mean[0][1] / count[0] - mean[1][1] / count[1]);
            score[hypothesis] = Math.abs(sep - expected);
        }
        if (score[0] == Double.MAX_VALUE && score[1] == Double.MAX_VALUE) {
            return elevationSum[0] / seen[0] >= elevationSum[1] / seen[1] ? 0 : 1;
        }
        return score[0] <= score[1] ? 0 : 1;
    }

    private boolean looksDownOnMap(List<Sighting> ours, int cell, Pose pose) {
        double[] upSpot = mapPoint(alliance, cell);
        double side = Math.signum(upSpot[0] - HIVE_CENTER_X_IN);
        double[] downSpot = {HIVE_CENTER_X_IN + side * DOWN_CELL_TAG_OFFSET_IN, upSpot[1]};
        double[] asUp = meanFieldPoint(ours, cell, UP_CELL_TAG_HEIGHT_IN, pose);
        double[] asDown = meanFieldPoint(ours, cell, DOWN_CELL_TAG_HEIGHT_IN, pose);
        if (asUp == null || asDown == null) return false;
        return Math.hypot(asDown[0] - downSpot[0], asDown[1] - downSpot[1])
                < Math.hypot(asUp[0] - upSpot[0], asUp[1] - upSpot[1]);
    }

    private double[] meanFieldPoint(List<Sighting> ours, int cell, double height, Pose pose) {
        double x = 0, y = 0;
        int n = 0;
        for (Sighting s : ours) {
            if (s.cell != cell) continue;
            double[] r = TargetMath.targetInRobotFrame(s.tx, s.ty, height, CAMERA_HEIGHT_IN,
                    CAMERA_PITCH_DEG, CAMERA_YAW_DEG, CAMERA_FORWARD_IN, CAMERA_LEFT_IN);
            if (r == null) continue;
            double[] f = TargetMath.robotToField(r, pose.x(), pose.y(), pose.heading());
            x += f[0];
            y += f[1];
            n++;
        }
        return n == 0 ? null : new double[]{x / n, y / n};
    }

    private void switchUpCell(int cell) {
        if (cell != currentUpCell) {
            sampleCount = 0;
            lastSeenMs = -1e12;
            currentUpCell = cell;
        }
    }

    private void checkAgainstMap(double[] center, int cell) {
        if (!USE_FIELD_MAP || !fieldLocalized || mapRejected) return;
        double[] map = mapPoint(alliance, cell);
        lastMapErrorIn = Math.hypot(center[0] - map[0], center[1] - map[1]);
        if (lastMapErrorIn <= MAP_AGREE_IN) {
            mapConfirmed = true;
            mapMismatches = 0;
        } else if (lastMapErrorIn > MAP_REJECT_IN) {
            if (++mapMismatches >= MAP_REJECT_COUNT) mapRejected = true;
        }
    }

    private double[] clusterCenter(List<double[]> points, int cell) {
        int n = points.size();
        double mx = 0, my = 0, mo = 0;
        for (double[] p : points) {
            mx += p[0];
            my += p[1];
            mo += p[2];
        }
        mx /= n;
        my /= n;
        mo /= n;

        if (n >= 2) {
            double ax = 0, ay = 0, var = 0;
            for (double[] p : points) {
                double d = p[2] - mo;
                ax += d * (p[0] - mx);
                ay += d * (p[1] - my);
                var += d * d;
            }
            double len = Math.hypot(ax, ay);
            if (var > 1e-6 && len > 1e-6) {
                double[] axis = {ax / len, ay / len};
                int k = axisIndex(cell);
                if (CELL_AXIS[k] == null) {
                    CELL_AXIS[k] = axis;
                } else {
                    double bx = 0.8 * CELL_AXIS[k][0] + 0.2 * axis[0];
                    double by = 0.8 * CELL_AXIS[k][1] + 0.2 * axis[1];
                    double bl = Math.hypot(bx, by);
                    if (bl > 1e-6) CELL_AXIS[k] = new double[]{bx / bl, by / bl};
                }
            }
        }

        double[] axis = CELL_AXIS[axisIndex(cell)];
        if (axis == null) return new double[]{mx, my};
        return new double[]{mx - mo * axis[0], my - mo * axis[1]};
    }
}