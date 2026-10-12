package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.SubsystemIF;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;

public class Shooter extends SubsystemIF {

    private final Telemetry telemetry;
    private final  HardwareMap hardwareMap;
    private double motorPower;
    private DcMotorEx rightShooterMotor;
    private DcMotorEx leftShooterMotor;
    private CRServo rightIntakeServo;
    private CRServo leftIntakeServo;

    public Shooter(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        motorPower = 0;

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
        rightIntakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        leftIntakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void enableShooter() {
        motorPower = SHOOTER_POWER;
    }

    public void startUpServos() {
        rightIntakeServo.setPower(SERVO_POWER);
        leftIntakeServo.setPower(SERVO_POWER);
    }

    public void stopShooter() {
        motorPower = 0;
        rightIntakeServo.setPower(SERVO_REST_POSITION);
        leftIntakeServo.setPower(SERVO_REST_POSITION);
    }

    @Override
    public void periodic() {
        rightShooterMotor.setPower(motorPower);
        leftShooterMotor.setPower(motorPower);
        telemetry.addData("Shooter Motor Power", motorPower);
    }
}