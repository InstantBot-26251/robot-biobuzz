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
    private boolean fixedRpmHeld = false;
    private double fixedRpmAdjust = 0;
    private DoubleSupplier distanceSupplier = () -> Double.NaN;
    private BooleanSupplier feedPermission = () -> true;
    private boolean forcedFeedAllowed = true;
    private DoubleSupplier fixedAutoRpm = () -> 0;
    private DoubleSupplier extraRpm = () -> 0;
    private boolean feedLatched = false;
    private boolean feedOverride = false;

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
    private double smoothedVoltage = NOMINAL_VOLTAGE;
    private boolean feedForced = false;
    private Boolean braking = null;
    private boolean encoderMismatch = false;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private final ElapsedTime feedRequestTimer = new ElapsedTime();
    private int loadedShotPoints = 0;
    private double rawLeftRpm = 0, rawRightRpm = 0;
    private double sideRpmDiff = 0;
    private boolean encoderDead = false;
    private final ElapsedTime noSpeedTimer = new ElapsedTime();

    public Shooter(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightShooterServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftShooterServo");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        voltage = voltageSensor.getVoltage();
        smoothedVoltage = voltage;
        loadedShotPoints = ShotTableStore.load();
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
        rightShooterMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftShooterMotor.setDirection(DcMotorEx.Direction.FORWARD);

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
        rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setFeedPermission(BooleanSupplier feedPermission) {
        this.feedPermission = feedPermission;
    }

    public void setForcedFeedAllowed(boolean forcedFeedAllowed) {
        this.forcedFeedAllowed = forcedFeedAllowed;
    }

    public void setExtraRpm(DoubleSupplier extraRpm) {
        this.extraRpm = extraRpm;
    }

    public void setFixedAutoRpm(DoubleSupplier fixedAutoRpm) {
        this.fixedAutoRpm = fixedAutoRpm;
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

    public void holdFixedRpm() {
        if (!isSpinning()) integral = 0;
        fixedRpmHeld = true;
        fixedRpmAdjust = 0;
    }

    public void releaseFixedRpm() {
        fixedRpmHeld = false;
        fixedRpmAdjust = 0;
    }

    public boolean isFixedRpmHeld() {
        return fixedRpmHeld;
    }

    public void adjustFixedRpm(double deltaRpm) {
        fixedRpmAdjust += deltaRpm;
    }

    private boolean isSpinning() {
        return mode != Mode.OFF || fixedRpmHeld;
    }

    public void adjustTrim(double deltaRpm) {
        trimRpm += deltaRpm;
    }

    public void resetTrim() {
        trimRpm = 0;
        fixedRpmAdjust = 0;
    }

    public void startUpServos() {
        if (!feedRequested) feedRequestTimer.reset();
        feedRequested = true;
    }

    public void stopShooter() {
        mode = Mode.OFF;
        feedRequested = false;
        feedOverride = false;
    }

    public void stopServos() {
        feedRequested = false;
        feedOverride = false;
    }

    // Runs the feed wheels no matter what the speed checks say (used for the timed auto feed)
    public void setFeedOverride(boolean on) {
        feedOverride = on;
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
        return isSpinning() && inTolerance && inToleranceTimer.milliseconds() >= READY_STABLE_MS;
    }

    private double readRpm() {

        double right = Math.abs(toRpm(rightShooterMotor.getVelocity()));
        double left = Math.abs(toRpm(leftShooterMotor.getVelocity()));
        rawRightRpm = right;
        rawLeftRpm = left;
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

    private void updateVoltage() {
        if (voltageTimer.milliseconds() > 250) {
            voltage = voltageSensor.getVoltage();
            smoothedVoltage += 0.15 * (voltage - smoothedVoltage);
            voltageTimer.reset();
        }
    }

    private double lowVoltageBoostRpm() {
        if (!VOLTAGE_COMPENSATION) return 0;
        return Math.max(0, LOW_VOLTAGE_THRESHOLD - smoothedVoltage) * LOW_VOLTAGE_RPM_PER_VOLT;
    }

    private double computeTarget() {
        if (fixedRpmHeld && mode != Mode.OPEN_LOOP) {
            usingDistance = false;
            return Math.max(MIN_RPM, Math.min(MAX_RPM, FIXED_RPM + fixedRpmAdjust));
        }
        switch (mode) {
            case MANUAL:
                usingDistance = false;
                return manualRpm;
            case AUTO:
                if (fixedAutoRpm.getAsDouble() > 0) {
                    usingDistance = false;
                    return fixedAutoRpm.getAsDouble();
                }
                double distance = distanceSupplier.getAsDouble();
                double rpm = Double.NaN;
                if (!Double.isNaN(distance)) {
                    rpm = ShotCalculator.rpmForDistance(distance, SHOT_DISTANCES_IN, SHOT_RPMS,
                            MAX_EXTRAPOLATE_IN, MIN_RPM, MAX_RPM);
                }
                usingDistance = !Double.isNaN(rpm);
                if (!usingDistance) rpm = DEFAULT_RPM;
                return Math.max(MIN_RPM, Math.min(MAX_RPM, rpm + SHOT_RPM_OFFSET + extraRpm.getAsDouble() + trimRpm + lowVoltageBoostRpm()));
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
        sideRpmDiff = alpha * sideRpmDiff + (1 - alpha) * (rawRightRpm - rawLeftRpm);

        targetRpm = computeTarget();
        double power = 0;

        if (mode == Mode.OPEN_LOOP) {
            updateVoltage();
            power = Math.max(0, Math.min(1, openLoopPower));
        } else if (targetRpm <= 0) {
            integral = 0;
        } else {
            updateVoltage();
            double compensation = VOLTAGE_COMPENSATION && voltage > 6 ? NOMINAL_VOLTAGE / voltage : 1.0;
            double error = targetRpm - measuredRpm;

            if (SHOOTER_I != 0) {
                if (Math.abs(error) < SPINUP_BOOST_RPM) integral += error * dt;
                double iLimit = SHOOTER_I_MAX_POWER / Math.abs(SHOOTER_I);
                integral = Math.max(-iLimit, Math.min(iLimit, integral));
            } else {
                integral = 0;
            }

            if (raw > ENCODER_DEAD_RPM || lastPower < 0.5) noSpeedTimer.reset();
            if (noSpeedTimer.milliseconds() > ENCODER_DEAD_MS) encoderDead = true;
            if (raw > ENCODER_DEAD_RPM) encoderDead = false;

            double feedforward = (SHOOTER_KV * targetRpm + SHOOTER_KS) * compensation;
            if (encoderDead) {

                integral = 0;
                power = feedforward;
            } else {
                power = feedforward + SHOOTER_P * error + SHOOTER_I * integral;
                if (error > SPINUP_BOOST_RPM) {
                    power = 1.0;
                } else if (error < -2 * SHOOTER_VELOCITY_TOLERANCE) {
                    power = -Math.abs(MAX_SLOWDOWN_POWER);
                }
                // During an auto volley, recover gently so the speed doesn't overshoot between balls
                if (feedLatched) power = Math.min(power, feedforward + VOLLEY_MAX_EXTRA_POWER);
            }
            power = Math.max(-Math.abs(MAX_SLOWDOWN_POWER), Math.min(1, power));
        }

        setBrake(!isSpinning());

        // In auto, nudge the two sides toward the same speed (plus the right-side trim)
        double balance = 0;
        if (!forcedFeedAllowed && mode == Mode.AUTO && power > 0 && !encoderDead && !encoderMismatch) {
            balance = SIDE_BALANCE_P * (AUTO_RIGHT_RPM_TRIM - sideRpmDiff);
            balance = Math.max(-SIDE_BALANCE_MAX_POWER, Math.min(SIDE_BALANCE_MAX_POWER, balance));
        }
        rightShooterMotor.setPower(Math.min(1, power + balance));
        leftShooterMotor.setPower(Math.min(1, power - balance));
        lastPower = power;

        boolean nowInTolerance = targetRpm > 0 && Math.abs(targetRpm - measuredRpm) <= SHOOTER_VELOCITY_TOLERANCE;
        if (nowInTolerance && !inTolerance) inToleranceTimer.reset();
        inTolerance = nowInTolerance;

        if (feedOverride && isSpinning()) {
            feeding = true;
        } else if (!feedRequested || !isSpinning()) {
            feeding = false;
            feedForced = false;
            feedLatched = false;
        } else if (feedLatched) {
            feeding = true;
        } else if (!GATE_FEED_ON_SPEED) {
            feeding = true;
        } else if (atTargetVelocity() && feedPermission.getAsBoolean()) {
            // In auto, once the first ball goes keep feeding until the volley ends
            if (!forcedFeedAllowed && mode == Mode.AUTO) feedLatched = true;
            feeding = true;
            feedForced = false;
        } else if (feedForced) {
            feeding = true;
        } else if (targetRpm - measuredRpm > 2 * SHOOTER_VELOCITY_TOLERANCE || !feedPermission.getAsBoolean()) {
            feeding = false;
        }

        if (forcedFeedAllowed && feedRequested && isSpinning() && !feeding && feedRequestTimer.milliseconds() > FEED_GATE_TIMEOUT_MS) {
            feeding = true;
            feedForced = true;
        }
        if (feeding && !feedForced) feedRequestTimer.reset();
        double servoPower = feeding ? (feedLatched ? VOLLEY_SERVO_POWER : SERVO_POWER) : 0;
        rightIntakeServo.setPower(servoPower);
        leftIntakeServo.setPower(servoPower);

        telemetry.addLine();
        telemetry.addData("Shooter Mode", fixedRpmHeld ? "FIXED RPM (left trigger)" : mode + (mode == Mode.AUTO ? (usingDistance ? " (Limelight)" : " (DEFAULT_RPM)") : ""));
        StringBuilder table = new StringBuilder();
        for (int i = 0; i < Math.min(SHOT_DISTANCES_IN.length, SHOT_RPMS.length); i++) {
            table.append(i == 0 ? "" : " | ")
                    .append(String.format(java.util.Locale.US, "%.0fin=%.0f", SHOT_DISTANCES_IN[i], SHOT_RPMS[i]));
        }
        telemetry.addData("Shot Table", (table.length() == 0 ? "EMPTY" : table.toString())
                + (loadedShotPoints > 0 ? " (from Shot Tuner file)" : " (from ShooterConstants)"));
        telemetry.addData("Shooter Target RPM", targetRpm);
        telemetry.addData("Shooter RPM", measuredRpm);
        telemetry.addData("Shooter Power", power);
        telemetry.addData("Shooter Battery (V)", String.format(java.util.Locale.US, "%.2f now, %.2f smoothed, low-voltage boost +%.0f RPM (%s)",
                voltage, smoothedVoltage, lowVoltageBoostRpm(), VOLTAGE_COMPENSATION ? "on" : "off"));
        telemetry.addData("Shooter Encoders L / R (RPM)", String.format(java.util.Locale.US, "%.0f / %.0f", rawLeftRpm, rawRightRpm));
        if (encoderDead) telemetry.addLine("Shooter: ENCODER READS 0 - running on feedforward only, check encoder cables");
        telemetry.addData("Shooter Trim RPM", trimRpm);
        telemetry.addData("Shooter Ready", atTargetVelocity());
        telemetry.addData("Shooter Feeding", feeding);
        if (encoderMismatch) telemetry.addLine("Shooter: ENCODER MISMATCH - check both shooter encoder cables");
    }
}
