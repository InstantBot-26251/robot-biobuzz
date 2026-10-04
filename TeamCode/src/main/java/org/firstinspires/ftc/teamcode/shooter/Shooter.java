package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.util.SubsystemIF;

public class Shooter extends SubsystemIF {
    public static final double DOOR_CLOSED_POSITION = 0.25;
    public static final double DOOR_OPEN_POSITION = 0.5; // TODO: tune on robot
    public static final long FLYWHEEL_SPIN_UP_MS = 750;

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private double power;
    private DcMotorSimple shooterMotor;
    private Servo intakeServo;

    public Shooter(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        shooterMotor = hardwareMap.get(DcMotorSimple.class, "shooterMotor");
        intakeServo = hardwareMap.get(Servo.class, "shooterServo");
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void teleopInit() {
        closeDoor();
        resetShotPower();
    }

    @Override
    public void periodic() {
        telemetry.addData("Shot Power: ", getPower());
    }


    public void openDoor() {
        intakeServo.setPosition(DOOR_OPEN_POSITION);
    }

    public void closeDoor() {
        intakeServo.setPosition(DOOR_CLOSED_POSITION);
    }

    public void spinUpFlywheel() {
        shooterMotor.setPower(power);
    }

    public void increasePower() {
        power = Math.min(1.0, power + 0.05);
        shooterMotor.setPower(power);
    }

    public void decreasePower() {
        power = Math.max(0.0, power - 0.05);
        shooterMotor.setPower(power);
    }

    public void resetShotPower() {
        power = 0.5;
        // 0.5 is just a magic number
        shooterMotor.setPower(power);
    }

    public void stopFlywheel() {
        shooterMotor.setPower(0);
    }

    public double getPower() {
        return shooterMotor.getPower();
    }
}