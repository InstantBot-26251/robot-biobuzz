package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.SubsystemIF;

public class Shooter extends SubsystemIF {
    public static final double SHOOTER_POWER = 0.5; // TODO: tune on robot

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private double motorPower;
    private DcMotorSimple shooterMotor;
    private Servo intakeServo;

    public Shooter(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        motorPower = 0;

        shooterMotor = hardwareMap.get(DcMotorSimple.class, "shooterMotor");
        intakeServo = hardwareMap.get(Servo.class, "shooterServo");
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

    public void stopShooter() {
        motorPower = 0;
    }

    @Override
    public void periodic() {
        shooterMotor.setPower(motorPower);

        telemetry.addData("Shooter Motor Power", motorPower);
    }
}