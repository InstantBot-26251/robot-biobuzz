package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "TeleOp Red")
public class RedTeleOp extends OpMode {
    private RobotContainer robot;

    @Override
    public void init() {
        Alliance.setCurrent(Alliance.RED);
        robot = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.teleopInit();
    }

    @Override
    public void init_loop() {
        telemetry.addData("Alliance", Alliance.getCurrent());
        telemetry.update();
    }

    @Override
    public void loop() {
        robot.periodic();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}