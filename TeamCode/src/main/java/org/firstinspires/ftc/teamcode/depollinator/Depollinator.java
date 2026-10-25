package org.firstinspires.ftc.teamcode.depollinator;

import static org.firstinspires.ftc.teamcode.depollinator.DepollinatorConstants.DEPOLLINATOR_SERVO_POSITION;
import static org.firstinspires.ftc.teamcode.depollinator.DepollinatorConstants.DEPOLLINATOR_SERVO_REST;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

public class Depollinator extends SubsystemIF {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Servo depollinatorServo;

    public Depollinator(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        this.depollinatorServo = hardwareMap.get(Servo.class, "depollinator");
    }

    @Override
    public void autonomousInit() {
        setUpServo();
    }

    @Override
    public void teleopInit() {
        setUpServo();
    }

    public void setUpServo() {
        depollinatorServo.setDirection(Servo.Direction.FORWARD);
    }

    public void depollinate() {
        depollinatorServo.setPosition(DEPOLLINATOR_SERVO_POSITION);
    }

    public void restDepollinator() {
        depollinatorServo.setPosition(DEPOLLINATOR_SERVO_REST);
    }

    @Override
    public void periodic() {
        telemetry.addData("Depollinator Servo Position", depollinatorServo.getPosition());
    }
}
