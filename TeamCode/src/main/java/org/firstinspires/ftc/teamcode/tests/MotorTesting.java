package org.firstinspires.ftc.teamcode.tests;
//Made by Nicolas

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class MotorTesting extends OpMode {
    private DcMotorEx motor;
    private TouchSensor touch;

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "1");
        touch = hardwareMap.get(TouchSensor.class, "touch");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if(touch.isPressed()) {
            motor.setPower(0);
        } else {
            motor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        }
    }
}