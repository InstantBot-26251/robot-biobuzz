package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.SubsystemIF;

public class Shooter extends SubsystemIF {
    public static final double SHOOTER_POWER = 0.5; // TODO: tune on robot
    public static double SERVO_POWER = 0.5; // TODO: tune the robot

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private double motorPower;
    private double servoPosition;
    private DcMotorEx rightShooterMotor;

    private DcMotorEx leftShooterMotor;
    private Servo rightIntakeServo;
    private Servo leftIntakeServo;

    private boolean isShooting = false;
    private long shootStartTime = 0;

    private static final long MOTOR_SPINUP_TIME = 1000; // 1 second (not sure if time is accurate)
    private static final long SERVO_INTAKE_TIME = 500;    // 0.5 seconds (not sure if time is accurate)

    public Shooter(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        motorPower = 0;

        rightShooterMotor = hardwareMap.get(DcMotorEx.class, "rightShooterMotor");
        leftShooterMotor = hardwareMap.get(DcMotorEx.class, "leftShooterMotor");
        rightIntakeServo = hardwareMap.get(Servo.class, "rightShooterServo");
        leftIntakeServo = hardwareMap.get(Servo.class, "leftShooterServo");
    }

    public void shoot() {
        long currentTime = System.currentTimeMillis();

        if (isShooting) return;

        // Start spinning
            motorPower = SHOOTER_POWER;
            servoPosition = 0;
            shootStartTime = currentTime;
            isShooting = true;
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void teleopInit() {

    }

    public void enableShooter() {
        motorPower = SHOOTER_POWER;
    }

    public void enableServo() {
        servoPosition = SERVO_POWER;
    }

    public void stopShooter() {
        motorPower = 0;
        servoPosition = 0;
        isShooting = false;
        shootStartTime = 0;
    }

    @Override
    public void periodic() {
        if (isShooting) {
            long currentTime = System.currentTimeMillis();
            long shootingSequenceTime = currentTime - shootStartTime;

            if (shootingSequenceTime >= MOTOR_SPINUP_TIME) {
                enableServo();
            }

            if (shootingSequenceTime >= MOTOR_SPINUP_TIME + SERVO_INTAKE_TIME) {
                stopShooter();
            }
        }

        rightShooterMotor.setPower(motorPower);
        leftShooterMotor.setPower(motorPower);
        rightIntakeServo.setPosition(servoPosition);
        leftIntakeServo.setPosition(servoPosition);

        telemetry.addData("Shooter Motor Power", motorPower);
        telemetry.addData("Servo Position", servoPosition);
    }
}