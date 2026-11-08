package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.vision.TargetMath;
import org.firstinspires.ftc.teamcode.vision.VisionConstants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@Configurable
@TeleOp(name = "Limelight Calibration", group = "Setup")
public class LimelightCalibration extends LinearOpMode {
    public static double[] DISTANCES_IN = {45, 70, 95};
    public static double COLLECT_SECONDS = 3;

    private Telemetry t;
    private Limelight3A limelight;
    private boolean lastA = false;

    @Override
    public void runOpMode() {
        t = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        try {
            limelight = hardwareMap.get(Limelight3A.class, VisionConstants.LIMELIGHT_NAME);
        } catch (Exception e) {
            t.addLine("No Limelight named '" + VisionConstants.LIMELIGHT_NAME + "' in the robot configuration.");
            t.update();
            waitForStart();
            return;
        }
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(VisionConstants.APRILTAG_PIPELINE);
        limelight.start();

        t.addLine("Limelight Calibration: finds the real camera height and pitch.");
        t.addLine("Hive tilted, robot facing it square. Press START.");
        t.update();
        waitForStart();

        List<double[]> sightings = new ArrayList<>();
        for (int spot = 0; spot < DISTANCES_IN.length && opModeIsActive(); spot++) {
            double distance = DISTANCES_IN[spot];
            while (opModeIsActive() && !aPressed()) {
                t.addLine(String.format(Locale.US, "SPOT %d of %d", spot + 1, DISTANCES_IN.length));
                t.addLine(String.format(Locale.US, "Put the robot so the floor distance from under the camera lens to under the middle of the HIGH tags is %.1f in.", distance));
                t.addLine("Face the hive square (camera pointing straight at the tags). Keep still, press A.");
                t.addData("High tags in view", highTags(limelight.getLatestResult()).size());
                t.update();
            }
            if (!opModeIsActive()) return;
            ElapsedTime timer = new ElapsedTime();
            long lastStamp = Long.MIN_VALUE;
            int frames = 0;
            while (opModeIsActive() && timer.seconds() < COLLECT_SECONDS) {
                LLResult result = limelight.getLatestResult();
                if (result != null && result.isValid() && result.getControlHubTimeStamp() != lastStamp) {
                    lastStamp = result.getControlHubTimeStamp();
                    List<double[]> high = highTags(result);
                    for (double[] tag : high) sightings.add(new double[]{tag[1], distance});
                    if (!high.isEmpty()) frames++;
                }
                t.addLine(String.format(Locale.US, "SPOT %d: collecting, keep still... %.1f s", spot + 1, COLLECT_SECONDS - timer.seconds()));
                t.addData("frames with high tags", frames);
                t.update();
            }
            if (frames < 10) {
                fail("Only " + frames + " frames saw the high tags at spot " + (spot + 1) + ". Check the hive is in view and run again.");
                return;
            }
        }

        double[] solved = TargetMath.solveCamera(sightings, VisionConstants.UP_CELL_TAG_HEIGHT_IN);
        while (opModeIsActive()) {
            if (Double.isNaN(solved[0])) {
                t.addLine("Could not solve. Check the distances and run again.");
            } else {
                t.addLine("DONE. Put these in vision/VisionConstants.java:");
                t.addLine(String.format(Locale.US, "public static double CAMERA_HEIGHT_IN = %.2f;", solved[0]));
                t.addLine(String.format(Locale.US, "public static double CAMERA_PITCH_DEG = %.2f;", solved[1]));
                t.addLine();
                t.addData("fit error (in)", String.format(Locale.US, "%.2f", solved[2]));
                if (solved[2] > 1.0) t.addLine("WARNING: fit error over 1 in. A distance was measured wrong or the robot wasn't square. Run again.");
                if (solved[1] < 10 || solved[1] > 60) t.addLine("WARNING: pitch looks wrong. Run again.");
            }
            t.update();
            sleep(50);
        }
    }

    private List<double[]> highTags(LLResult result) {
        List<double[]> out = new ArrayList<>();
        if (result == null || !result.isValid()) return out;
        double[][] sum = new double[12][3];
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        for (LLResultTypes.FiducialResult f : tags) {
            int group = (f.getFiducialId() - 30) / 4;
            if (f.getFiducialId() < 30 || group >= 4) continue;
            sum[group][0] += f.getTargetYDegreesNoCrosshair();
            sum[group][1]++;
        }
        int best = -1;
        for (int g = 0; g < 4; g++) {
            if (sum[g][1] == 0) continue;
            if (best < 0 || sum[g][0] / sum[g][1] > sum[best][0] / sum[best][1]) best = g;
        }
        if (best < 0) return out;
        for (LLResultTypes.FiducialResult f : tags) {
            if (f.getFiducialId() < 30 || (f.getFiducialId() - 30) / 4 != best) continue;
            out.add(new double[]{f.getTargetXDegreesNoCrosshair(), f.getTargetYDegreesNoCrosshair()});
        }
        return out;
    }

    private boolean aPressed() {
        boolean a = gamepad1.a;
        boolean pressed = a && !lastA;
        lastA = a;
        return pressed;
    }

    private void fail(String message) {
        while (opModeIsActive()) {
            t.addLine("CALIBRATION STOPPED");
            t.addLine(message);
            t.update();
            sleep(50);
        }
    }
}
