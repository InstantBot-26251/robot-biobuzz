package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class RobotDriving1 extends OpMode {
    private DcMotorEx motor1;
    private DcMotorEx motor2;
    private DcMotorEx motor3;
    private DcMotorEx motor4;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotorEx.class, "1");
        motor2 = hardwareMap.get(DcMotorEx.class, "2");
        motor3 = hardwareMap.get(DcMotorEx.class, "3");
        motor4 = hardwareMap.get(DcMotorEx.class, "4");

        motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor1.setDirection(DcMotorSimple.Direction.REVERSE);

        motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor2.setDirection(DcMotorSimple.Direction.REVERSE);

        motor3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor3.setDirection(DcMotorSimple.Direction.REVERSE);

        motor4.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor4.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
            motor1.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            motor2.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            motor3.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
            motor4.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
    }
}
