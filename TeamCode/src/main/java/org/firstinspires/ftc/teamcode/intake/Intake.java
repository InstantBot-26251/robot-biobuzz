package org.firstinspires.ftc.teamcode.intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

public class Intake extends SubsystemIF {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private DcMotorEx motor;

    public Intake(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
        this.motor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void setPower(double power) {
        motor.setPower(power);
    }

    public void stop() {
        motor.setPower(0);
    }

    @Override
    public void autonomousInit() {
    }

    @Override
    public void teleopInit() {
    }
}