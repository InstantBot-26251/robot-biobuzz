package org.firstinspires.ftc.teamcode.shooter;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;

public class Shooter extends SubsystemIF {
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private DcMotorEx rightShooterMotor;
    private DcMotorEx leftShooterMotor;
    private CRServo rightIntakeServo;
    private CRServo leftIntakeServo;
    private double motorPower;
    private boolean shooterEnabled;

    public Shooter(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        motorPower = SHOOTER_POWER;
        shooterEnabled = false;

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

    public void enableShooter() {
        shooterEnabled = true;
    }

    public void setMotorPower(double power) {
        motorPower = power;
    }

    public void startUpServos() {
        rightIntakeServo.setPower(SERVO_POWER);
        leftIntakeServo.setPower(SERVO_POWER);
    }

    public void stopShooter() {
        shooterEnabled = false;
        rightIntakeServo.setPower(0);
        leftIntakeServo.setPower(0);
    }

    public void stopServos() {
        rightIntakeServo.setPower(0);
        leftIntakeServo.setPower(0);
    }

    @Override
    public void periodic() {
        double power = shooterEnabled ? motorPower : 0;
        rightShooterMotor.setPower(power);
        leftShooterMotor.setPower(power);
        telemetry.addData("Shooter Motor Power ", power);
    }
}
