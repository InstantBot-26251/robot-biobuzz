package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.SubsystemIF;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.DEFAULT_RPM;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.FEED_GATE_TIMEOUT_MS;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.GATE_FEED_ON_SPEED;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.MAX_EXTRAPOLATE_IN;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.MAX_RPM;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.MAX_SLOWDOWN_POWER;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.MIN_RPM;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.NOMINAL_VOLTAGE;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.READY_STABLE_MS;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.SHOOTER_I_MAX_POWER;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.SHOOTER_KS;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.SHOOTER_KV;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.SHOT_DISTANCES_IN;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.SHOT_RPMS;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.SPINUP_BOOST_RPM;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.VELOCITY_FILTER;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.VELOCITY_SOURCE;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.toRpm;

public class Shooter extends SubsystemIF {
    public enum Mode { OFF, AUTO, MANUAL, OPEN_LOOP }

    private final Telemetry telemetry;
    private final  HardwareMap hardwareMap;
    private DcMotorEx rightShooterMotor;
    private DcMotorEx leftShooterMotor;
    private CRServo rightIntakeServo;
    private CRServo leftIntakeServo;
    private VoltageSensor voltageSensor;

    private Mode mode = Mode.OFF;
    private double manualRpm = DEFAULT_RPM;
    private double openLoopPower = 0;
    private double trimRpm = 0;
    private DoubleSupplier distanceSupplier = () -> Double.NaN;
    private BooleanSupplier feedPermission = () -> true;

    private double targetRpm = 0;
    private double measuredRpm = 0;
    private double integral = 0;
    private double lastPower = 0;
    private boolean feedRequested = false;
    private boolean feeding = false;
    private boolean usingDistance = false;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime inToleranceTimer = new ElapsedTime();
    private boolean inTolerance = false;
    private double voltage = NOMINAL_VOLTAGE;
    private Boolean braking = null;
    private boolean encoderMismatch = false;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime feedRequestTimer = new ElapsedTime();

    public Shooter(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightShooterServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftShooterServo");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();
    }

    @Override
    public void autonomousInit() {
        setUpMotors();
        setUpServos();
    }

    @Override
    public void teleopInit() {
        setUpMotors();
        setUpServos();
    }

    public void setUpMotors() {
        rightShooterMotor.setDirection(DcMotorEx.Direction.FORWARD);
        leftShooterMotor.setDirection(DcMotorEx.Direction.REVERSE);

        rightShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        setBrake(true);
    }

    private void setBrake(boolean brake) {
        if (braking != null && braking == brake) return;
        DcMotor.ZeroPowerBehavior behavior = brake ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        rightShooterMotor.setZeroPowerBehavior(behavior);
        leftShooterMotor.setZeroPowerBehavior(behavior);
        braking = brake;
    }

    public void setUpServos() {
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void setFeedPermission(BooleanSupplier feedPermission) {
        this.feedPermission = feedPermission;
    }

    public void setDistanceSupplier(DoubleSupplier distanceSupplier) {
        this.distanceSupplier = distanceSupplier;
    }

    public void enableShooter() {
        if (mode == Mode.OFF) integral = 0;
        mode = Mode.AUTO;
    }

    public void setOpenLoopPower(double power) {
        openLoopPower = power;
        mode = Mode.OPEN_LOOP;
    }

    public double getVoltage() {
        return voltage;
    }

    public void setManualRpm(double rpm) {
        if (mode == Mode.OFF) integral = 0;
        manualRpm = rpm;
        mode = Mode.MANUAL;
    }

    public void adjustTrim(double deltaRpm) {
        trimRpm += deltaRpm;
    }

    public void resetTrim() {
        trimRpm = 0;
    }

    public void startUpServos() {
        if (!feedRequested) feedRequestTimer.reset();
        feedRequested = true;
    }

    public void stopShooter() {
        mode = Mode.OFF;
        feedRequested = false;
    }

    public void stopServos() {
        feedRequested = false;
    }

    public Mode getMode() {
        return mode;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getShooterRpm() {
        return measuredRpm;
    }

    public double getShooterVelocity() {
        return toTicksPerSec(measuredRpm);
    }

    public double getLastPower() {
        return lastPower;
    }

    public boolean atTargetVelocity() {
        return mode != Mode.OFF && inTolerance && inToleranceTimer.milliseconds() >= READY_STABLE_MS;
    }

    private double readRpm() {
        double right = toRpm(rightShooterMotor.getVelocity());
        double left = toRpm(leftShooterMotor.getVelocity());
        switch (VELOCITY_SOURCE) {
            case LEFT:
                return left;
            case AVERAGE:
                double high = Math.max(left, right);
                double low = Math.min(left, right);
                encoderMismatch = high > 500 && low < 0.5 * high;
                return encoderMismatch ? high : 0.5 * (left + right);
            case RIGHT:
            default:
                return right;
        }
    }

    private double computeTarget() {
        switch (mode) {
            case MANUAL:
                usingDistance = false;
                return manualRpm;
            case AUTO:
                double distance = distanceSupplier.getAsDouble();
                double rpm = Double.NaN;
                if (!Double.isNaN(distance)) {
                    rpm = ShotCalculator.rpmForDistance(distance, SHOT_DISTANCES_IN, SHOT_RPMS,
                            MAX_EXTRAPOLATE_IN, MIN_RPM, MAX_RPM);
                }
                usingDistance = !Double.isNaN(rpm);
                if (!usingDistance) rpm = DEFAULT_RPM;
                return Math.max(MIN_RPM, Math.min(MAX_RPM, rpm + trimRpm));
            case OFF:
            default:
                usingDistance = false;
                return 0;
        }
    }

    @Override
    public void periodic() {
        double dt = Math.min(loopTimer.seconds(), 0.1);
        loopTimer.reset();

        double raw = readRpm();
        double alpha = Math.max(0, Math.min(0.95, VELOCITY_FILTER));
        measuredRpm = alpha * measuredRpm + (1 - alpha) * raw;

        targetRpm = computeTarget();
        double power = 0;

        if (mode == Mode.OPEN_LOOP) {
            if (voltageTimer.milliseconds() > 250) {
                voltage = voltageSensor.getVoltage();
                voltageTimer.reset();
            }
            power = Math.max(0, Math.min(1, openLoopPower));
        } else if (targetRpm <= 0) {
            integral = 0;
        } else {
            if (voltageTimer.milliseconds() > 250) {
                voltage = voltageSensor.getVoltage();
                voltageTimer.reset();
            }
            double compensation = voltage > 6 ? NOMINAL_VOLTAGE / voltage : 1.0;
            double error = targetRpm - measuredRpm;

            if (SHOOTER_I != 0) {
                integral += error * dt;
                double iLimit = SHOOTER_I_MAX_POWER / Math.abs(SHOOTER_I);
                integral = Math.max(-iLimit, Math.min(iLimit, integral));
            } else {
                integral = 0;
            }

            power = (SHOOTER_KV * targetRpm + SHOOTER_KS) * compensation
                    + SHOOTER_P * error
                    + SHOOTER_I * integral;

            if (error > SPINUP_BOOST_RPM) {
                power = 1.0;
            } else if (error < -2 * SHOOTER_VELOCITY_TOLERANCE) {
                power = -Math.abs(MAX_SLOWDOWN_POWER);
            }
            power = Math.max(-Math.abs(MAX_SLOWDOWN_POWER), Math.min(1, power));
        }

        setBrake(mode == Mode.OFF);

        rightShooterMotor.setPower(power);
        leftShooterMotor.setPower(power);
        lastPower = power;

        boolean nowInTolerance = targetRpm > 0 && Math.abs(targetRpm - measuredRpm) <= SHOOTER_VELOCITY_TOLERANCE;
        if (nowInTolerance && !inTolerance) inToleranceTimer.reset();
        inTolerance = nowInTolerance;

        if (!feedRequested || mode == Mode.OFF) {
            feeding = false;
        } else if (!GATE_FEED_ON_SPEED || feedRequestTimer.milliseconds() > FEED_GATE_TIMEOUT_MS) {
            feeding = true;
        } else if (atTargetVelocity() && feedPermission.getAsBoolean()) {
            feeding = true;
        } else if (targetRpm - measuredRpm > 2 * SHOOTER_VELOCITY_TOLERANCE || !feedPermission.getAsBoolean()) {
            feeding = false;
        }
        double servoPower = feeding ? SERVO_POWER : 0;
        rightIntakeServo.setPower(servoPower);
        leftIntakeServo.setPower(servoPower);

        telemetry.addLine();
        telemetry.addData("Shooter Mode", mode + (mode == Mode.AUTO ? (usingDistance ? " (Limelight)" : " (DEFAULT_RPM)") : ""));
        telemetry.addData("Shooter Target RPM", targetRpm);
        telemetry.addData("Shooter RPM", measuredRpm);
        telemetry.addData("Shooter Power", power);
        telemetry.addData("Shooter Trim RPM", trimRpm);
        telemetry.addData("Shooter Ready", atTargetVelocity());
        telemetry.addData("Shooter Feeding", feeding);
        if (encoderMismatch) telemetry.addLine("Shooter: ENCODER MISMATCH - check both shooter encoder cables");
    }
}
