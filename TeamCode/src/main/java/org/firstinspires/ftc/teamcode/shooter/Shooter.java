package org.firstinspires.ftc.teamcode.shooter;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

public class Shooter extends SubsystemIF {

    private final Telemetry telemetry;
    private final  HardwareMap hardwareMap;
    private DcMotorEx rightShooterMotor;
    private DcMotorEx leftShooterMotor;
    private CRServo rightIntakeServo;
    private CRServo leftIntakeServo;
    private final PIDFController shooterPID;


    public Shooter(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        shooterPID = new PIDFController(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);


        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightShooterServo");
        leftIntakeServo = hardwareMap.get(CRServo.class, "leftShooterServo");
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

        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setUpServos() {
        rightIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void startUpServos() {
        rightIntakeServo.setPower(0.75);
        leftIntakeServo.setPower(0.75);
    }

    public void stopShooter() {
        shooterPID.setSetPoint(0);
        rightIntakeServo.setPower(0);
        leftIntakeServo.setPower(0);
    }

    public void stopServos() {
        rightIntakeServo.setPower(0);
        leftIntakeServo.setPower(0);
    }

    public double getShooterVelocity() {
        return rightShooterMotor.getVelocity();
    }

    public boolean atTargetVelocity() {
        return shooterPID.getSetPoint() != 0
                && Math.abs(getShooterVelocity() - shooterPID.getSetPoint()) <= toTicksPerSec(SHOOTER_VELOCITY_TOLERANCE);
    }

    @Override
    public void periodic() {
        shooterPID.setPIDF(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F); // TODO: delete once the PID is tuned

        double velocity = getShooterVelocity();
        double output = 0;

        if (shooterPID.getSetPoint() == 0) {
            rightShooterMotor.setPower(0);
            leftShooterMotor.setPower(0);
        } else {
            output = shooterPID.calculate(velocity) / toTicksPerSec(MAX_ACHIEVABLE_RPM);
            rightShooterMotor.setPower(output);
            leftShooterMotor.setPower(output);
        }

        telemetry.addData("Shooter Target Velocity", shooterPID.getSetPoint());
        telemetry.addData("Shooter Velocity", velocity);
        telemetry.addData("Shooter PID Output", output);
    }
}