package org.firstinspires.ftc.teamcode.depollinator;

import static org.firstinspires.ftc.teamcode.depollinator.DepollinatorConstants.DEPOLLINATOR_ARM_OUT;
import static org.firstinspires.ftc.teamcode.depollinator.DepollinatorConstants.DEPOLLINATOR_ARM_REST;
import static org.firstinspires.ftc.teamcode.depollinator.DepollinatorConstants.DEPOLLINATOR_WHEEL_POWER;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

public class Depollinator extends SubsystemIF {
    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private Servo armServo;
    private CRServo wheelServo;
    private double armPosition;
    private double wheelPower;

    public Depollinator(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        armPosition = DEPOLLINATOR_ARM_REST;
        wheelPower = 0;

        this.armServo = hardwareMap.get(Servo.class, "dePollinator1");
        this.wheelServo = hardwareMap.get(CRServo.class, "dePollinator2");
    }

    @Override
    public void autonomousInit() {
        setUpServos();
    }

    @Override
    public void teleopInit() {
        setUpServos();
    }

    public void setUpServos() {
        armServo.setDirection(Servo.Direction.FORWARD);
        wheelServo.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void depollinate() {
        armPosition = DEPOLLINATOR_ARM_OUT;
        wheelPower = DEPOLLINATOR_WHEEL_POWER;
    }

    public void restDepollinator() {
        armPosition = DEPOLLINATOR_ARM_REST;
        wheelPower = 0;
    }

    @Override
    public void periodic() {
        armServo.setPosition(armPosition);
        wheelServo.setPower(wheelPower);

        telemetry.addData("Depollinator Arm Position", armPosition);
        telemetry.addData("Depollinator Wheel Power", wheelPower);
    }
}
