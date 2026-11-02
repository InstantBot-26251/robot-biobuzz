package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.OTOSConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.pedro.OtosMath;
import org.firstinspires.ftc.teamcode.vision.TargetMath;
import java.util.Locale;

@Configurable
@TeleOp(name = "OTOS Calibration", group = "Setup")
public class OTOSCalibration extends LinearOpMode {
    public static double PUSH_DISTANCE_IN = 48;
    public static int SPIN_TURNS = 5;

    private static final double SCALAR_MIN = 0.872;
    private static final double SCALAR_MAX = 1.127;

    private Telemetry t;
    private Mecanum drive;
    private boolean lastA = false;
    private String name;

    @Override
    public void runOpMode() throws InterruptedException {
        t = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        drive = new Mecanum(hardwareMap, Constants.teleOpConfig);
        name = Constants.localizerConfig.name.get();

        t.addLine("OTOS Calibration. Robot on the floor, clear space around it.");
        t.addLine("Press START, then follow the steps. A = start / finish a step.");
        t.update();
        waitForStart();

        OTOSLocalizer otos = startStep("STEP 1/5: ANGULAR SCALAR",
                "Keep still, press A. Then spin " + SPIN_TURNS + " full turns COUNTER-CLOCKWISE, stop, press A.",
                1.0, 1.0, 0, 0, 0);
        if (otos == null) return;
        double prev = otos.pose().heading();
        double total = 0;
        while (opModeIsActive() && !aPressed()) {
            driveWithSticks(true);
            otos.update();
            double h = otos.pose().heading();
            total += TargetMath.normalizeAngle(h - prev);
            prev = h;
            t.addLine("STEP 1/5: spin " + SPIN_TURNS + " turns CCW, stop, press A");
            t.addData("turns so far", total / (2 * Math.PI));
            t.update();
        }
        stopDrive();
        if (!opModeIsActive()) return;
        if (Math.abs(total) < Math.PI) {
            fail("Barely any rotation measured. Is the OTOS plugged in and named \"" + name + "\"?");
            return;
        }
        boolean headingInverted = total < 0;
        double angularScalar = SPIN_TURNS * 2 * Math.PI / Math.abs(total);

        otos = startStep("STEP 2/5: LINEAR SCALAR + MOUNT ANGLE",
                "Line the robot up on a tile seam. Keep still, press A. Then PUSH it straight FORWARD exactly "
                        + PUSH_DISTANCE_IN + " in without turning, stop, press A.",
                1.0, angularScalar, 0, 0, 0);
        if (otos == null) return;
        while (opModeIsActive() && !aPressed()) {
            stopDrive();
            otos.update();
            t.addLine("STEP 2/5: push forward " + PUSH_DISTANCE_IN + " in, stop, press A");
            t.addData("measured x", otos.pose().x());
            t.addData("measured y", otos.pose().y());
            t.update();
        }
        if (!opModeIsActive()) return;
        Pose pushed = otos.pose();
        double pushedDistance = Math.hypot(pushed.x(), pushed.y());
        if (pushedDistance < 5) {
            fail("Only " + String.format(Locale.US, "%.1f", pushedDistance) + " in measured for a " + PUSH_DISTANCE_IN + " in push. "
                    + "The OTOS isn't seeing the floor. Check: (1) the clear protective film/cover is peeled off the sensor, "
                    + "(2) the sensor is about 10 mm (0.4 in) above the tiles - after a rebuild it's often too high, "
                    + "(3) the lens is clean. This is what makes autos drive forever.");
            return;
        }
        double linearScalar = PUSH_DISTANCE_IN / pushedDistance;
        if (linearScalar > 1.5) {
            fail(String.format(Locale.US, "Measured %.1f in for a %.1f in push (%.0fx too little). The OTOS is losing tracking: "
                    + "it is probably too high off the floor (needs ~10 mm) or the cover film is still on. "
                    + "A scalar can't fix this; the chip only accepts 0.872-1.127.",
                    pushedDistance, PUSH_DISTANCE_IN, linearScalar));
            return;
        }
        double mountHeading = OtosMath.mountHeading(pushed.x(), pushed.y());

        double[] spin1 = spinStep("STEP 3/5: OFFSET SPIN #1", linearScalar, angularScalar, 0, 0, mountHeading);
        if (spin1 == null) return;
        double[] sensor = OtosMath.sensorPosition(spin1[0], spin1[1], 0, spin1[2]);
        if (sensor == null) {
            fail("Turn at least 90 degrees (180 is best). Restart this OpMode.");
            return;
        }

        double[] offset;
        if (OtosMath.abs(sensor) < 0.3) {
            offset = new double[]{0, 0};
        } else {
            double[] spin2 = spinStep("STEP 4/5: OFFSET SPIN #2", linearScalar, angularScalar, sensor[0], sensor[1], mountHeading);
            if (spin2 == null) return;
            double[] g = OtosMath.conventionRotation(sensor, sensor, spin2[0], spin2[1], 0, spin2[2]);
            if (g == null) {
                fail("Turn at least 90 degrees (180 is best). Restart this OpMode.");
                return;
            }
            offset = OtosMath.solveOffset(sensor, g);
        }

        otos = startStep("STEP 5/5: VERIFY",
                "Keep still, press A. Then turn in place a few times. Drift should stay under ~1 in. Press A when done.",
                linearScalar, angularScalar, offset[0], offset[1], mountHeading);
        if (otos == null) return;
        double maxDrift = 0;
        while (opModeIsActive() && !aPressed()) {
            driveWithSticks(true);
            otos.update();
            maxDrift = Math.max(maxDrift, Math.hypot(otos.pose().x(), otos.pose().y()));
            t.addLine("STEP 5/5: turn in place, press A when done");
            t.addData("drift now (in)", Math.hypot(otos.pose().x(), otos.pose().y()));
            t.addData("max drift (in)", maxDrift);
            t.update();
        }
        stopDrive();

        while (opModeIsActive()) {
            t.addLine("DONE. Copy into pedro/Constants.java -> localizerConfig:");
            t.addLine("c.linearScalar.set(" + linearScalar + ");");
            t.addLine("c.angularScalar.set(" + angularScalar + ");");
            t.addLine("c.offset.set(new Pose(" + offset[0] + ", " + offset[1] + ", " + mountHeading + "));");
            t.addLine();
            t.addData("mount angle (deg)", Math.toDegrees(mountHeading));
            t.addData("verify max drift (in)", maxDrift);
            if (linearScalar < SCALAR_MIN || linearScalar > SCALAR_MAX) {
                t.addLine("WARNING: linear scalar is outside the OTOS range 0.872-1.127. Re-measure the push; if it");
                t.addLine("repeats, the sensor is the wrong height off the floor (needs ~10 mm).");
            }
            if (angularScalar < SCALAR_MIN || angularScalar > SCALAR_MAX) {
                t.addLine("WARNING: angular scalar is outside the OTOS range 0.872-1.127. Count the turns again.");
            }
            if (headingInverted) {
                t.addLine("WARNING: heading went DOWN while turning counter-clockwise. If the robot really turned");
                t.addLine("CCW, the OTOS is mounted upside down. It must face the floor.");
            }
            if (maxDrift > 2) {
                t.addLine("WARNING: drift over 2 in. Spin slower / more in place and run again.");
            }
            t.update();
            sleep(50);
        }
    }

    private double[] spinStep(String title, double linear, double angular, double ox, double oy, double mount) {
        OTOSLocalizer otos = startStep(title,
                "Keep still, press A. Then TURN IN PLACE about 180 degrees (right stick), stop, press A.",
                linear, angular, ox, oy, mount);
        if (otos == null) return null;
        while (opModeIsActive() && !aPressed()) {
            driveWithSticks(true);
            otos.update();
            t.addLine(title + ": turn ~180 in place, stop, press A");
            t.addData("heading (deg)", Math.toDegrees(otos.pose().heading()));
            t.update();
        }
        stopDrive();
        if (!opModeIsActive()) return null;
        Pose p = otos.pose();
        return new double[]{p.x(), p.y(), p.heading()};
    }

    private OTOSLocalizer startStep(String title, String instructions, double linear, double angular,
                                    double ox, double oy, double mount) {
        while (opModeIsActive() && !aPressed()) {
            stopDrive();
            t.addLine(title);
            t.addLine(instructions);
            t.update();
        }
        if (!opModeIsActive()) return null;
        t.addLine(title + ": calibrating gyro, keep still...");
        t.update();
        OTOSLocalizer otos = new OTOSLocalizer(hardwareMap, new OTOSConfig(c -> {
            c.name.set(name);
            c.linearUnit.set(DistanceUnit.INCH);
            c.linearScalar.set(linear);
            c.angularScalar.set(angular);
            c.offset.set(new Pose(ox, oy, mount));
        }));
        otos.setPose(Pose.zero());
        otos.update();
        return otos;
    }

    private void driveWithSticks(boolean turnOnly) {
        double turn = -gamepad1.right_stick_x * 0.35;
        double fwd = turnOnly ? 0 : -gamepad1.left_stick_y * 0.4;
        double left = turnOnly ? 0 : -gamepad1.left_stick_x * 0.4;
        drive.drive(new DrivePowers(fwd, left, turn), true);
    }

    private void stopDrive() {
        drive.stop(false);
    }

    private boolean aPressed() {
        boolean a = gamepad1.a;
        boolean pressed = a && !lastA;
        lastA = a;
        return pressed;
    }

    private void fail(String message) {
        stopDrive();
        while (opModeIsActive()) {
            t.addLine("CALIBRATION STOPPED");
            t.addLine(message);
            t.update();
            sleep(50);
        }
    }
}
