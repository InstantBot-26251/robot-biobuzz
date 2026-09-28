package org.firstinspires.ftc.teamcode.Intake;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

public class Intake extends SubsystemIF {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;


    @Override
    public void autonomousInit() {

    }

    @Override
    public void teleopInit() {

    }

    public Intake(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;
    }
}
