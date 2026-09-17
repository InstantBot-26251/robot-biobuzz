package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Testing", group = "TeleOp")
public class ServoTesting extends LinearOpMode {

    private Servo servo;
    private static final double SMALL_STEP = 0.01;
    private static final double LARGE_STEP = 0.1;
    private double position = 0.5;
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;

    @Override
    public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setPosition(position);

        waitForStart();

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;

            if (left && !lastLeft) {
                position -= SMALL_STEP;
            }
            if (right && !lastRight) {
                position += SMALL_STEP;
            }
            if (up && !lastUp) {
                position -= LARGE_STEP;
            }
            if (down && !lastDown) {
                position += LARGE_STEP;
            }

            position = Math.max(0.0, Math.min(1.0, position));
            servo.setPosition(position);

            lastUp = up;
            lastDown = down;
            lastLeft = left;
            lastRight = right;

            telemetry.addData("Servo Position", position);
            telemetry.update();
        }
    }
}