package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

public class Intake extends SubsystemIF {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private double motorPower;
    private DcMotorEx intakeMotor;

    public Intake(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        motorPower = 0;

        this.intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void teleopInit() {
    }

    public void intake() {
        setPower(1.0);
    }

    public void outtake() {
        setPower(-1.0);
    }

    public void stop() {
        setPower(0);
    }

    public void noIntake() {
        setPower(-.25);
    }

    private void setPower(double power) {
        motorPower = power;
    }

    @Override
    public void periodic() {
        intakeMotor.setPower(motorPower);

        telemetry.addData("Intake Motor Power", motorPower);
    }
}