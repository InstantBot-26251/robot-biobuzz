package org.firstinspires.ftc.teamcode.defloweringStick;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;
import static org.firstinspires.ftc.teamcode.defloweringStick.DefloweringStickConstants.DEFLOWERING_ANGLE;
import static org.firstinspires.ftc.teamcode.defloweringStick.DefloweringStickConstants.DEFLOWERING_REST_ANGLE;
import static org.firstinspires.ftc.teamcode.defloweringStick.DefloweringStickConstants.DEFLOWERING_SERVO_POSITION;
import static org.firstinspires.ftc.teamcode.defloweringStick.DefloweringStickConstants.DEFLOWERING_SERVO_REST;

import com.pedropathing.utils.Angle;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;
import org.firstinspires.ftc.teamcode.defloweringStick.DefloweringStickConstants.*;
import org.firstinspires.ftc.teamcode.defloweringStick.DefloweringStickCommands.*;

public class DefloweringStick extends SubsystemIF {

    private Telemetry telemetry;
    private HardwareMap hardwareMap;
    private double defloweringAngle;
    private Servo defloweringServo;

    public DefloweringStick(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        this.defloweringServo = hardwareMap.get(Servo.class, "i");
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
        defloweringServo.setDirection(Servo.Direction.FORWARD);
    }

    public void deflower() {
        defloweringServo.setPosition(DEFLOWERING_SERVO_POSITION);
        defloweringAngle = AngleUnit.normalizeDegrees(DEFLOWERING_ANGLE);
    }

    public void restDeflowerer() {
        defloweringServo.setPosition(DEFLOWERING_SERVO_REST);
        defloweringAngle = AngleUnit.normalizeDegrees(DEFLOWERING_REST_ANGLE);
    }

    @Override
    public void periodic() {
        telemetry.addData("Deflowering Angle", defloweringAngle);
        telemetry.addData("Deflowering Servo Position", defloweringServo);

        if(gamepad2.x) {
            deflower();
        } else {
            restDeflowerer();
        }

    }
}
