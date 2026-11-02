package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.opmodes.Alliance;
import org.firstinspires.ftc.teamcode.util.PanelsDrawing;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.teamcode.vision.VisionConstants.*;

public class Limelight extends SubsystemIF {
    private final Telemetry telemetry;
    private final Chassis chassis;
    private final HiveTracker tracker = new HiveTracker();
    private Limelight3A limelight;
    private String error = null;

    private final ElapsedTime clock = new ElapsedTime();
    private long lastFrameStamp = Long.MIN_VALUE;
    private double lastLatencyMs = Double.NaN;
    private boolean fieldLocalized = false;

    private static final int PIPELINE_COUNT = 10;
    private static final double PIPELINE_SEARCH_MS = 400;
    private static int foundPipeline = -1;
    private int activePipeline = APRILTAG_PIPELINE;
    private boolean pipelineSearchDone = false;
    private final ElapsedTime pipelineTimer = new ElapsedTime();

    public Limelight(Telemetry telemetry, HardwareMap hardwareMap, Chassis chassis) {
        this.telemetry = telemetry;
        this.chassis = chassis;
        try {
            limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        } catch (Exception e) {
            limelight = null;
            error = "No device named \"" + LIMELIGHT_NAME + "\" in the robot configuration";
        }
    }

    @Override
    public void autonomousInit() {
        start(true);
    }

    @Override
    public void teleopInit() {
        start(RobotContainer.ROBOT_POSE != null && RobotContainer.POSE_FROM_AUTO);
    }

    private void start(boolean fieldLocalized) {
        this.fieldLocalized = fieldLocalized;
        tracker.reset(Alliance.getCurrent(), fieldLocalized);
        if (limelight == null) return;
        limelight.setPollRateHz(100);
        activePipeline = foundPipeline >= 0 ? foundPipeline : APRILTAG_PIPELINE;
        pipelineSearchDone = false;
        limelight.pipelineSwitch(activePipeline);
        pipelineTimer.reset();
        limelight.start();
    }

    // Only leave a pipeline that isn't an AprilTag one, and give up after one full lap.
    // Seeing no tags right now is normal, so that alone never moves us off a pipeline.
    private void checkPipeline(LLResult result) {
        if (foundPipeline >= 0 || pipelineSearchDone) return;
        if (result.getPipelineIndex() != activePipeline) {
            pipelineTimer.reset();
            return;
        }
        String type = result.getPipelineType();
        boolean aprilTagType = type != null && type.toLowerCase().contains("fiducial");
        if (!result.getFiducialResults().isEmpty() || aprilTagType) {
            foundPipeline = activePipeline;
            return;
        }
        if (type == null || type.isEmpty() || pipelineTimer.milliseconds() < PIPELINE_SEARCH_MS) return;
        activePipeline = (activePipeline + 1) % PIPELINE_COUNT;
        if (activePipeline == APRILTAG_PIPELINE) pipelineSearchDone = true;
        limelight.pipelineSwitch(activePipeline);
        pipelineTimer.reset();
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    public void clear() {
        tracker.reset(Alliance.getCurrent(), fieldLocalized);
    }

    public HiveTracker getTracker() {
        return tracker;
    }

    public boolean hasTarget() {
        return tracker.hasTarget(clock.milliseconds());
    }

    public boolean isLive() {
        return tracker.isLive(clock.milliseconds());
    }

    public double getDistance(Pose robot) {
        double[] t = tracker.target(clock.milliseconds());
        if (t == null) return Double.NaN;
        return Math.hypot(t[0] - robot.x(), t[1] - robot.y());
    }

    public double getAimFieldHeading(Pose robot) {
        double[] t = tracker.target(clock.milliseconds());
        if (t == null) return robot.heading();
        return Math.atan2(t[1] - robot.y(), t[0] - robot.x());
    }

    public double getAimErrorDeg(Pose robot) {
        if (!hasTarget()) return Double.NaN;
        return Math.toDegrees(TargetMath.normalizeAngle(getAimFieldHeading(robot) - robot.heading()));
    }

    @Override
    public void periodic() {
        String raw = "no Limelight device";
        if (limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result == null) {
                raw = "NO RESULT (connected " + limelight.isConnected() + ", running " + limelight.isRunning() + ")";
            } else {
                StringBuilder ids = new StringBuilder();
                for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
                    ids.append(ids.length() == 0 ? "" : ",").append(f.getFiducialId());
                }
                checkPipeline(result);
                raw = "pipeline " + result.getPipelineIndex() + " (" + result.getPipelineType() + ")"
                        + (foundPipeline >= 0 ? " LOCKED" : pipelineSearchDone ? " NO APRILTAG PIPELINE FOUND" : " checking pipeline")
                        + " | valid " + result.isValid()
                        + " | age " + result.getStaleness() + " ms | tag IDs " + (ids.length() == 0 ? "none" : ids)
                        + " | connected " + limelight.isConnected() + ", running " + limelight.isRunning();
            }
            if (result != null && !result.getFiducialResults().isEmpty()
                    && result.getStaleness() <= MAX_FRAME_AGE_MS
                    && result.getControlHubTimeStamp() != lastFrameStamp) {
                lastFrameStamp = result.getControlHubTimeStamp();
                process(result);
            }
        }

        double now = clock.milliseconds();
        Pose pose = chassis.getPose();
        double[] target = tracker.target(now);
        if (target != null) PanelsDrawing.drawTarget(target[0], target[1], tracker.isLive(now));

        telemetry.addLine();
        if (error != null) telemetry.addData("Limelight", error);
        telemetry.addData("LL RAW", raw + " | alliance " + Alliance.getCurrent());
        telemetry.addData("LL Target Source", tracker.source(now));
        telemetry.addData("LL Up Cell", tracker.cellName(tracker.getCurrentUpCell())
                + (SIMPLE_TRACKING ? " (simple)" : tracker.getLastUsedRange() ? " (3D)" : " (angle only)"));
        telemetry.addData("LL Alliance Tags / Up Tags", tracker.getLastAllianceTags() + " / " + tracker.getLastUpTags());
        telemetry.addData("LL Note", tracker.getLastNote());
        telemetry.addData("LL Tag Heights (in)", tracker.getLastTagHeights()
                + (USE_TAG_RANGE ? "" : " (ignored: USE_TAG_RANGE off)"));
        double measuredHeight = HiveTracker.measuredUpHeightAboveCamera();
        telemetry.addData("LL Up Tags Above Camera (in)", Double.isNaN(measuredHeight)
                ? "measuring (needs 2+ tags of a cell in view)"
                : String.format(java.util.Locale.US, "%.1f  -> real camera height ~%.1f in (CAMERA_HEIGHT_IN is %.1f)",
                        measuredHeight, UP_CELL_TAG_HEIGHT_IN - measuredHeight, CAMERA_HEIGHT_IN));
        telemetry.addData("LL Distance (in)", getDistance(pose));
        telemetry.addData("LL Aim Error (deg)", getAimErrorDeg(pose));
        telemetry.addData("LL Latency (ms)", lastLatencyMs);
        telemetry.addData("LL Field Map", !USE_FIELD_MAP ? "off"
                : tracker.isMapRejected() ? "DISAGREES with camera -> disabled"
                  : tracker.isMapConfirmed() ? String.format("confirmed (%.1f in off)", tracker.getLastMapErrorIn())
                    : tracker.mapUsable() ? "in use, not yet confirmed" : "not used (pose not from auto)");
    }

    private void process(LLResult result) {
        List<HiveTracker.Tag> tags = new ArrayList<>();
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            double tx = f.getTargetXDegreesNoCrosshair();
            double ty = f.getTargetYDegreesNoCrosshair();
            if (tx == 0 && ty == 0) {
                tx = f.getTargetXDegrees();
                ty = f.getTargetYDegrees();
            }
            double range = Double.NaN;
            Pose3D camSpace = f.getTargetPoseCameraSpace();
            if (camSpace != null && camSpace.getPosition() != null) {
                Position p = camSpace.getPosition().toUnit(DistanceUnit.INCH);
                range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            }
            tags.add(new HiveTracker.Tag(f.getFiducialId(), tx, ty, range));
        }

        lastLatencyMs = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
        Pose capturePose = chassis.poseAt(System.nanoTime() - (long) (lastLatencyMs * 1e6));
        tracker.update(tags, capturePose, clock.milliseconds());
    }
}