package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.SubsystemIF;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;

public class Shooter extends SubsystemIF {


    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private double motorPower;
    private double servoPosition;
    private DcMotorEx rightShooterMotor;
    private DcMotorEx leftShooterMotor;
    private Servo rightIntakeServo;
    private Servo leftIntakeServo;

    public Shooter(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        motorPower = 0;

        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightIntakeServo = hardwareMap.get(Servo.class, "rightShooterServo");
        leftIntakeServo = hardwareMap.get(Servo.class, "leftShooterServo");
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
        rightShooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setUpServos() {
        rightIntakeServo.setDirection(Servo.Direction.FORWARD);
        rightIntakeServo.setDirection(Servo.Direction.REVERSE);
    }

    public void enableShooter() {
        motorPower = SHOOTER_POWER;
    }

    public void startUpServos() {
        servoPosition = SERVO_POWER;
        rightIntakeServo.setPosition(servoPosition);
        leftIntakeServo.setPosition(servoPosition);
    }

    public void stopShooter() {
        motorPower = 0;
        servoPosition = 0;
    }

    @Override
    public void periodic() {
        rightShooterMotor.setPower(motorPower);
        leftShooterMotor.setPower(motorPower);
        telemetry.addData("Shooter Motor Power", motorPower);
        telemetry.addData("Servo Position", servoPosition);
    }
}