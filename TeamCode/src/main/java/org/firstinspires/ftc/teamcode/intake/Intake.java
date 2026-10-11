package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        setupMotors();
    }

    @Override
    public void teleopInit() {
        setupMotors();
    }

    public void setupMotors() {
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void intake() {
        setPower(IntakeConstants.INTAKE_POWER);
    }

    public void outtake() {
        setPower(IntakeConstants.OUTTAKE_POWER);
    }

    public void stop() {
        setPower(IntakeConstants.STOP_INTAKE_POWER);
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