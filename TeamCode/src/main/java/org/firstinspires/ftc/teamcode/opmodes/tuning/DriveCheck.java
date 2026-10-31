package org.firstinspires.ftc.teamcode.opmodes.tuning;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.OTOSLocalizer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.vision.TargetMath;

@TeleOp(name = "Drive Check", group = "Setup")
public class DriveCheck extends OpMode {
    private Telemetry t;
    private Mecanum drive;
    private OTOSLocalizer otos;
    private DcMotorEx fl, fr, bl, br;
    private MecanumConfig config;

    private Pose lastPose;
    private long lastNanos;
    private String forwardResult = "untested", strafeResult = "untested", turnResult = "untested";
    private double fwdScore = 0, strScore = 0, turnScore = 0;
    private boolean wasTesting = false;

    @Override
    public void init() {
        t = new JoinedTelemetry(telemetry, PanelsTelemetry.INSTANCE.getFtcTelemetry());
        config = Constants.teleOpConfig;
        drive = new Mecanum(hardwareMap, config);
        fl = hardwareMap.get(DcMotorEx.class, config.frontLeftName.get());
        fr = hardwareMap.get(DcMotorEx.class, config.frontRightName.get());
        bl = hardwareMap.get(DcMotorEx.class, config.backLeftName.get());
        br = hardwareMap.get(DcMotorEx.class, config.backRightName.get());
        otos = new OTOSLocalizer(hardwareMap, Constants.localizerConfig);
        otos.setPose(Pose.zero());
        t.addLine("Drive Check ready. Constants.MOTORS_MIRRORED_LEFT_RIGHT = " + Constants.MOTORS_MIRRORED_LEFT_RIGHT);
        t.update();
    }

    @Override
    public void start() {
        otos.update();
        lastPose = otos.pose();
        lastNanos = System.nanoTime();
    }

    @Override
    public void loop() {
        otos.update();
        if (gamepad1.a) otos.setPose(Pose.zero());

        double wheel = 0.3;
        String testing = null;
        if (gamepad1.dpad_up) testing = "FRONT LEFT";
        else if (gamepad1.dpad_right) testing = "FRONT RIGHT";
        else if (gamepad1.dpad_down) testing = "BACK RIGHT";
        else if (gamepad1.dpad_left) testing = "BACK LEFT";

        double cmdFwd = -gamepad1.left_stick_y;
        double cmdLeft = -gamepad1.left_stick_x;
        double cmdTurn = -gamepad1.right_stick_x;

        if (testing != null) {
            if (!wasTesting) drive.stop(true);
            fl.setPower(gamepad1.dpad_up ? wheel : 0);
            fr.setPower(gamepad1.dpad_right ? wheel : 0);
            br.setPower(gamepad1.dpad_down ? wheel : 0);
            bl.setPower(gamepad1.dpad_left ? wheel : 0);
        } else {
            if (wasTesting) {
                fl.setPower(0);
                fr.setPower(0);
                bl.setPower(0);
                br.setPower(0);
            }
            if (Math.abs(cmdFwd) < 0.05 && Math.abs(cmdLeft) < 0.05 && Math.abs(cmdTurn) < 0.05) {
                drive.stop(false);
            } else {
                drive.drive(new DrivePowers(cmdFwd * 0.5, cmdLeft * 0.5, cmdTurn * 0.4), true);
            }
        }
        wasTesting = testing != null;

        Pose pose = otos.pose();
        long now = System.nanoTime();
        double dt = (now - lastNanos) / 1e9;
        if (lastPose != null && dt > 0) {
            double dx = pose.x() - lastPose.x();
            double dy = pose.y() - lastPose.y();
            double h = pose.heading();
            double vFwd = (Math.cos(h) * dx + Math.sin(h) * dy) / dt;
            double vLeft = (-Math.sin(h) * dx + Math.cos(h) * dy) / dt;
            double vTurn = TargetMath.normalizeAngle(pose.heading() - lastPose.heading()) / dt;

            if (testing == null) {
                if (Math.abs(cmdFwd) > 0.5 && Math.abs(cmdLeft) < 0.2 && Math.abs(cmdTurn) < 0.2 && Math.abs(vFwd) > 5) {
                    fwdScore += Math.signum(cmdFwd * vFwd);
                    forwardResult = fwdScore > 0 ? "OK" : "REVERSED";
                }
                if (Math.abs(cmdLeft) > 0.5 && Math.abs(cmdFwd) < 0.2 && Math.abs(cmdTurn) < 0.2 && Math.abs(vLeft) > 5) {
                    strScore += Math.signum(cmdLeft * vLeft);
                    strafeResult = strScore > 0 ? "OK" : "REVERSED";
                }
                if (Math.abs(cmdTurn) > 0.5 && Math.abs(cmdFwd) < 0.2 && Math.abs(cmdLeft) < 0.2 && Math.abs(vTurn) > 0.3) {
                    turnScore += Math.signum(cmdTurn * vTurn);
                    turnResult = turnScore > 0 ? "OK" : "REVERSED";
                }
            }
        }
        lastPose = pose;
        lastNanos = now;

        t.addLine("WHEELS (on blocks): d-pad UP=FL, RIGHT=FR, DOWN=BR, LEFT=BL");
        t.addLine("  the named wheel must spin FORWARD; no other wheel may move");
        t.addData("  testing", testing == null ? "-" : testing);
        t.addLine();
        t.addLine("DRIVE (on floor): left stick up, left stick right, right stick right");
        t.addData("  forward", forwardResult);
        t.addData("  strafe", strafeResult);
        t.addData("  turn", turnResult);
        t.addLine("  If strafe AND turn say REVERSED but forward is OK: flip");
        t.addLine("  MOTORS_MIRRORED_LEFT_RIGHT in pedro/Constants.java.");
        t.addLine("  (Also check with your eyes: stick right = robot goes right / turns clockwise.)");
        t.addLine("  turn + the d-pad test are always valid; forward/strafe need the OTOS calibrated first");
        t.addLine();
        t.addLine("OTOS (A = zero): push fwd -> X up, push left -> Y up, CCW -> heading up");
        t.addLine("  PUSH TEST: press A, push the robot forward 24 in by hand:");
        t.addLine("   X ~ +24 = good | X barely moves = OTOS can't see floor (height/cover film)");
        t.addLine("   X goes negative or Y moves instead = mount angle -> run OTOS Calibration");
        t.addData("  X", pose.x());
        t.addData("  Y", pose.y());
        t.addData("  Heading (deg)", Math.toDegrees(pose.heading()));
        t.addData("  motor names used", config.frontLeftName.get() + ", " + config.frontRightName.get()
                + ", " + config.backLeftName.get() + ", " + config.backRightName.get());
        t.update();
    }

    @Override
    public void stop() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        drive.stop(true);
    }
}
