package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;

@TeleOp(name = "TeleOp")
public class DriverControlled extends OpMode {
    private RobotContainer robot;

    @Override
    public void init() {
        robot = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.teleopInit();
    }

    @Override
    public void loop() {
        robot.periodic();
    }

}
