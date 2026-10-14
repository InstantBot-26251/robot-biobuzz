package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.vision.VisionConstants.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

public class Vision extends SubsystemIF {
    private static final long RESET_SMOOTHING_AFTER_MS = 500;

    private final Telemetry telemetry;
    private final Limelight3A limelight;

    private final ElapsedTime timeSinceTarget = new ElapsedTime();
    private boolean everSawTarget = false;
    private double distanceIn = Double.NaN;
    private int targetCell = HiveTargeting.NO_CELL;
    private String status = "Starting";
    private String tagReport = "";
    private int activePipeline = -1;

    public Vision(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.limelight = hardwareMap.tryGet(Limelight3A.class, "limelight");
    }

    @Override
    public void autonomousInit() {
        startLimelight();
    }

    @Override
    public void teleopInit() {
        startLimelight();
    }

    private void startLimelight() {
        if (limelight == null) {
            return;
        }
        activePipeline = Alliance.getCurrent() == Alliance.RED ? RED_PIPELINE : BLUE_PIPELINE;
        limelight.pipelineSwitch(activePipeline);
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public boolean hasTarget(long maxAgeMs) {
        return everSawTarget && timeSinceTarget.milliseconds() <= maxAgeMs;
    }

    public double getDistanceInches() {
        return distanceIn;
    }

    private void update() {
        if (limelight == null) {
            status = "No \"limelight\" in robot config";
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid() || result.getStaleness() > MAX_RESULT_AGE_MS
                || result.getFiducialResults() == null) {
            status = "No tags";
            tagReport = "";
            return;
        }

        Alliance alliance = Alliance.getCurrent();
        List<HiveTargeting.TagPosition> tags = new ArrayList<>();
        StringBuilder report = new StringBuilder();
        boolean missing3d = false;

        for (LLResultTypes.FiducialResult fiducial : result.getFiducialResults()) {
            int id = fiducial.getFiducialId();
            Pose3D pose = fiducial.getTargetPoseCameraSpace();
            if (pose == null) {
                missing3d = true;
                continue;
            }
            Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
            double range = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
            if (range < 1.0) {
                missing3d = true;
                continue;
            }

            HiveTargeting.TagPosition tag = HiveTargeting.locate(id, range,
                    fiducial.getTargetXDegrees(), fiducial.getTargetYDegrees(),
                    CAMERA_HEIGHT_IN, CAMERA_PITCH_DEG, CAMERA_FORWARD_OF_SHOOTER_IN);
            tags.add(tag);

            String owner = HiveTargeting.cellFor(alliance, id) == HiveTargeting.NO_CELL ? " other" : "";
            report.append(String.format(Locale.US, "%d:%.0fin%s  ", id, tag.heightIn, owner));
        }
        tagReport = report.toString();

        HiveTargeting.Choice choice = HiveTargeting.chooseUpCell(alliance, tags,
                UP_CELL_MIN_TAG_HEIGHT_IN, AIM_POINT_CLOSER_THAN_TAGS_IN);

        if (choice.cell == HiveTargeting.NO_CELL) {
            status = missing3d ? "Turn on Full 3D in the Limelight pipeline" : "No up CELL of ours in view";
            return;
        }

        if (!everSawTarget || timeSinceTarget.milliseconds() > RESET_SMOOTHING_AFTER_MS || Double.isNaN(distanceIn)) {
            distanceIn = choice.distanceIn;
        } else {
            distanceIn += SMOOTHING * (choice.distanceIn - distanceIn);
        }
        targetCell = choice.cell;
        everSawTarget = true;
        timeSinceTarget.reset();
        status = String.format(Locale.US, "Up CELL (%s) at %.1f in",
                targetCell == HiveTargeting.AUDIENCE_CELL ? "audience side" : "far side", distanceIn);
    }

    @Override
    public void periodic() {
        update();

        telemetry.addLine();
        telemetry.addData("Vision", status);
        telemetry.addData("Alliance", "%s (pipeline %d)", Alliance.getCurrent(), activePipeline);
        telemetry.addData("Tag heights", tagReport.isEmpty() ? "none" : tagReport);
    }
}
