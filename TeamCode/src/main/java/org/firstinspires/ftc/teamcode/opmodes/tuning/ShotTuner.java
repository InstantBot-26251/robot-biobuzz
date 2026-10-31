package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.shooter.ShotTableStore;
import org.firstinspires.ftc.teamcode.opmodes.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.AllianceStore;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

@TeleOp(name = "Shot Tuner", group = "Setup")
public class ShotTuner extends OpMode {
    private static final double OPEN_LOOP_TEST_POWER = 0.8;

    private RobotContainer robot;
    private Telemetry t;
    private Shooter shooter;
    private double tunerRpm = ShooterConstants.DEFAULT_RPM;
    private boolean spinning = false;
    private final List<double[]> points = new ArrayList<>();
    private boolean pA, pB, pY, pBack, pUp, pDown, pLeft, pRight, pRB, pLB;
    private String status = "";
    private final ElapsedTime statusTimer = new ElapsedTime();

    @Override
    public void init() {
        AllianceStore.load();
        robot = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.teleopInit();
        t = robot.getTelemetry();
        shooter = robot.getShooter();
        for (int i = 0; i < Math.min(ShooterConstants.SHOT_DISTANCES_IN.length, ShooterConstants.SHOT_RPMS.length); i++) {
            points.add(new double[]{ShooterConstants.SHOT_DISTANCES_IN[i], ShooterConstants.SHOT_RPMS[i]});
        }
    }

    @Override
    public void init_loop() {
        if (gamepad1.x) AllianceStore.save(Alliance.BLUE);
        if (gamepad1.b) AllianceStore.save(Alliance.RED);
        t.addLine("X = BLUE, B = RED");
        t.addData("Alliance", Alliance.getCurrent());
        robot.initLoop();
    }

    @Override
    public void start() {
        if (RobotContainer.ROBOT_POSE == null) robot.getChassis().resetHeading();
        robot.getLimelight().clear();
    }

    private void showStatus(String message) {
        status = message;
        statusTimer.reset();
    }

    @Override
    public void loop() {
        boolean a = gamepad1.a, b = gamepad1.b, y = gamepad1.y, back = gamepad1.back;
        boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down, left = gamepad1.dpad_left, right = gamepad1.dpad_right;
        boolean rb = gamepad1.right_bumper, lb = gamepad1.left_bumper;

        if (up && !pUp) tunerRpm += 50;
        if (down && !pDown) tunerRpm -= 50;
        if (right && !pRight) tunerRpm += 250;
        if (left && !pLeft) tunerRpm -= 250;
        tunerRpm = Math.max(0, Math.min(ShooterConstants.MAX_ACHIEVABLE_RPM, tunerRpm));

        if (a && !pA) spinning = true;
        if (b && !pB) {
            spinning = false;
            shooter.stopShooter();
        }

        if (lb) {
            shooter.setOpenLoopPower(OPEN_LOOP_TEST_POWER);
        } else if (pLB) {
            if (spinning) shooter.setManualRpm(tunerRpm); else shooter.stopShooter();
        } else if (spinning) {
            shooter.setManualRpm(tunerRpm);
        }

        if (rb && !pRB) shooter.startUpServos();
        if (!rb && pRB) shooter.stopServos();

        double distance = robot.getLimelight().getDistance(robot.getChassis().getPose());
        if (y && !pY) {
            if (Double.isNaN(distance)) {
                showStatus("!! NOT SAVED: Limelight doesn't see your hive. Turn until distance shows a number.");
                gamepad1.rumble(100);
            } else {
                points.add(new double[]{distance, tunerRpm});
                applyTable();
                showStatus(String.format(Locale.US, "SAVED point %d: %.1f in -> %.0f RPM (kept after restart)",
                        points.size(), distance, tunerRpm));
                gamepad1.rumbleBlips(2);
            }
        }
        if (back && !pBack && !points.isEmpty()) {
            points.remove(points.size() - 1);
            applyTable();
            showStatus("Deleted last point. " + points.size() + " left.");
        }

        pA = a; pB = b; pY = y; pBack = back; pUp = up; pDown = down; pLeft = left; pRight = right; pRB = rb; pLB = lb;

        t.addLine("===== SHOT TUNER =====");
        if (!status.isEmpty() && statusTimer.seconds() < 4) t.addLine(">> " + status);
        t.addLine("A spin | B stop | dpad +/-50 +/-250 | RB shoot | Y save | BACK undo");
        t.addData("Tuner RPM", tunerRpm);
        t.addData("Limelight distance (in)", distance);
        t.addData("RPM the table would pick", ShooterConstants.SHOT_RPMS.length == 0 ? Double.NaN :
                org.firstinspires.ftc.teamcode.shooter.ShotCalculator.rpmForDistance(distance,
                        ShooterConstants.SHOT_DISTANCES_IN, ShooterConstants.SHOT_RPMS,
                        ShooterConstants.MAX_EXTRAPOLATE_IN,
                        ShooterConstants.MIN_RPM, ShooterConstants.MAX_RPM));
        t.addData("Points saved", points.size() + (points.size() < 4 ? "  (get 4-6, closest to farthest)" : ""));
        if (lb) {
            double rpm = shooter.getShooterRpm();
            double kv = rpm > 100 ? OPEN_LOOP_TEST_POWER * (shooter.getVoltage() / ShooterConstants.NOMINAL_VOLTAGE) / rpm : Double.NaN;
            t.addData("FF TEST measured RPM", rpm);
            t.addData("FF TEST -> SHOOTER_KV", kv);
            t.addLine("(wait until RPM stops climbing, then copy SHOOTER_KV)");
        }
        t.addLine("--- SAVED POINTS (stored on the Control Hub) ---");
        if (ShooterConstants.SHOT_DISTANCES_IN.length == 0) t.addLine("  none yet - press Y to save one");
        for (int i = 0; i < Math.min(ShooterConstants.SHOT_DISTANCES_IN.length, ShooterConstants.SHOT_RPMS.length); i++) {
            t.addLine(String.format(Locale.US, "  %d)  %.1f in  ->  %.0f RPM",
                    i + 1, ShooterConstants.SHOT_DISTANCES_IN[i], ShooterConstants.SHOT_RPMS[i]));
        }
        t.addLine("===== END SHOT TUNER =====");

        robot.periodic();
    }

    private void applyTable() {
        List<double[]> sorted = new ArrayList<>(points);
        sorted.sort((p, q) -> Double.compare(p[0], q[0]));
        double[] d = new double[sorted.size()];
        double[] r = new double[sorted.size()];
        for (int i = 0; i < sorted.size(); i++) {
            d[i] = sorted.get(i)[0];
            r[i] = sorted.get(i)[1];
        }
        ShooterConstants.SHOT_DISTANCES_IN = d;
        ShooterConstants.SHOT_RPMS = r;
        if (!ShotTableStore.save(d, r)) showStatus("!! Could not write " + ShotTableStore.path());
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
