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
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;

import java.util.List;

public class Shooter extends SubsystemIF {


    private final Telemetry telemetry;
    private final  HardwareMap hardwareMap;
    private DcMotorEx rightShooterMotor;
    private DcMotorEx leftShooterMotor;
    private CRServo rightIntakeServo;
    private CRServo leftIntakeServo;
    private double motorPower;
    private boolean shooterEnabled;
    private double flywheelPower;
    private String powerSource = "manual";

    private final List<VoltageSensor> voltageSensors;
    private final ElapsedTime voltageTimer = new ElapsedTime();
    private double batteryVoltage = 12.0;


    public Shooter(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        motorPower = 0;
        shooterEnabled = false;
        flywheelPower = SHOOTER_POWER;
        voltageSensors = hardwareMap.getAll(VoltageSensor.class);


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


    public void setFlywheelPower(double power, String source) {
        flywheelPower = power;
        powerSource = source;
    }


    public double getBatteryVoltage() {
        return batteryVoltage;
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


    private void updateBatteryVoltage() {
        if (voltageTimer.milliseconds() < 250) {
            return;
        }
        voltageTimer.reset();
        for (VoltageSensor sensor : voltageSensors) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                batteryVoltage = voltage;
                return;
            }
        }
    }


    @Override
    public void periodic() {
        updateBatteryVoltage();

        motorPower = shooterEnabled ? flywheelPower : 0;
        rightShooterMotor.setPower(motorPower);
        leftShooterMotor.setPower(motorPower);
        telemetry.addData("Shooter Motor Power ", motorPower);
        telemetry.addData("Shooter Power Source", "%s (%.2f)", powerSource, flywheelPower);
        telemetry.addData("Battery", "%.2f V", batteryVoltage);
    }
}
