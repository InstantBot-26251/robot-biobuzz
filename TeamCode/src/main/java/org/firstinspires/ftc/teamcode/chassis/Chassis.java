package org.firstinspires.ftc.teamcode.chassis;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.opmodes.Alliance;
import org.firstinspires.ftc.teamcode.util.PanelsDrawing;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;
import org.firstinspires.ftc.teamcode.vision.TargetMath;

import static org.firstinspires.ftc.teamcode.chassis.DriveConstants.*;

public class Chassis extends SubsystemIF {
    private static final int POSE_HISTORY_SIZE = 64;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Follower follower;

    private final long[] historyTimes = new long[POSE_HISTORY_SIZE];
    private final Pose[] historyPoses = new Pose[POSE_HISTORY_SIZE];
    private int historyHead = 0;
    private int historyCount = 0;

    private double lastAimError = 0;

    private final RunawayGuard guard = new RunawayGuard();
    private boolean localizationFault = false;
    private String faultReason = "";

    public Chassis(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }

    @Override
    public void autonomousInit() {
        follower = Constants.createAutonomous(hardwareMap);
    }

    @Override
    public void teleopInit() {
        follower = Constants.createTeleop(hardwareMap);
        if (RobotContainer.ROBOT_POSE == null) {
            setPose(new Pose(0, 0, Alliance.getCurrent().driverForwardHeading()));
        } else {
            setPose(RobotContainer.ROBOT_POSE);
        }
    }

    public Pose getPose() {
        return follower.pose();
    }

    public boolean isFollowingPath() {
        return follower.following();
    }

    public boolean isPathFinished() {
        return !follower.isBusy() || !(isFollowingPath() || follower.holding());
    }

    public void followPath(Path path) {
        if (localizationFault) {
            follower.stop();
            return;
        }
        follower.follow(path);
    }

    public boolean hasLocalizationFault() {
        return localizationFault;
    }

    public void setPose(Pose pose) {
        follower.setPose(pose);
        historyCount = 0;
    }

    public void stopFollower() {
        follower.stop();
    }

    public void setHoldEnd(boolean holdEnd) {
        follower.holdEnd.set(holdEnd);
    }

    public void holdCurrentPose() {
        if (localizationFault) {
            follower.stop();
            return;
        }
        follower.hold(getPose());
    }

    public void holdPose(Pose pose) {
        if (localizationFault) {
            follower.stop();
            return;
        }
        follower.hold(pose);
    }

    public void resetHeading() {
        follower.setHeading(Alliance.getCurrent().driverForwardHeading());
        historyCount = 0;
    }

    public double driverHeading() {
        return follower.pose().heading() - Alliance.getCurrent().driverForwardHeading();
    }

    public void setDrivePowers(double forward, double left, double turn) {
        DrivePowers powers = ManualDrive.fieldCentric(forward, left, turn, driverHeading());
        follower.manual(powers);
    }

    public void setDrivePowersFacing(double forward, double left, double fieldHeading) {
        setDrivePowers(forward, left, headingLockTurn(fieldHeading));
    }

    public double headingLockTurn(double fieldHeading) {
        double error = TargetMath.normalizeAngle(fieldHeading - follower.pose().heading());
        lastAimError = error;
        double omega = follower.velocity().omega;
        double turn = AIM_P * error - AIM_D * omega;
        if (Math.abs(error) > Math.toRadians(AIM_TOLERANCE_DEG)) {
            turn += Math.signum(error) * AIM_KS;
        }
        return Math.max(-AIM_MAX_TURN, Math.min(AIM_MAX_TURN, turn));
    }

    public double getLastAimErrorDeg() {
        return Math.toDegrees(lastAimError);
    }

    public double getAngularVelocity() {
        return follower.velocity().omega;
    }

    public Pose poseAt(long nanoTime) {
        if (historyCount == 0) return getPose();
        Pose best = null;
        long bestDiff = Long.MAX_VALUE;
        for (int i = 0; i < historyCount; i++) {
            int idx = (historyHead - 1 - i + POSE_HISTORY_SIZE) % POSE_HISTORY_SIZE;
            long diff = Math.abs(historyTimes[idx] - nanoTime);
            if (diff < bestDiff) {
                bestDiff = diff;
                best = historyPoses[idx];
            }
        }
        return best == null ? getPose() : best;
    }

    private void runawayGuard() {
        if (localizationFault) return;
        Pose pose = follower.pose();
        double cmdForward = 0, cmdLeft = 0;
        try {
            java.util.Map<String, Object> d = follower.drivetrain.debug();
            cmdForward = ((Number) d.get("forward")).doubleValue();
            cmdLeft = ((Number) d.get("strafe")).doubleValue();
        } catch (Exception ignored) {
        }
        String reason = guard.update(System.nanoTime(), pose.x(), pose.y(), pose.heading(),
                follower.following() || follower.holding(), cmdForward, cmdLeft);
        if (reason != null) {
            localizationFault = true;
            faultReason = reason;
            follower.stop();
        }
    }

    @Override
    public void periodic() {
        follower.update();
        runawayGuard();

        historyTimes[historyHead] = System.nanoTime();
        historyPoses[historyHead] = follower.pose();
        historyHead = (historyHead + 1) % POSE_HISTORY_SIZE;
        historyCount = Math.min(historyCount + 1, POSE_HISTORY_SIZE);

        Pose pose = follower.pose();
        if (DRAW_FIELD) {
            PanelsDrawing.drawRobot(pose, Alliance.getCurrent() == Alliance.RED ? "#F44336" : "#2196F3");
        }

        telemetry.addLine();
        if (localizationFault) {
            telemetry.addLine("!!! LOCALIZATION FAULT - robot stopped: " + faultReason);
            telemetry.addLine("!!! Run Drive Check, then OTOS Calibration.");
        }
        telemetry.addData("Pose x", pose.x());
        telemetry.addData("Pose y", pose.y());
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading()));
        telemetry.addData("Is Following Path", isFollowingPath());
    }
}
