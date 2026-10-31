package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.opmodes.Alliance;
import org.firstinspires.ftc.teamcode.opmodes.AllianceStore;

@TeleOp(name = "TeleOp")
public class DriverControlled extends OpMode {
    private RobotContainer robot;

    @Override
    public void init() {
        AllianceStore.load();
        robot = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.teleopInit();
    }

    @Override
    public void init_loop() {
        robot.getTelemetry().addLine("ALLIANCE: " + Alliance.getCurrent()
                + "   (wrong? stop and run \"Set Alliance: " + (Alliance.getCurrent() == Alliance.RED ? "BLUE" : "RED") + "\")");
        robot.getTelemetry().addData("Pose from auto", RobotContainer.ROBOT_POSE == null ? "none (press START when facing away from you)" : "yes");
        robot.initLoop();
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
