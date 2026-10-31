package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;

@TeleOp(name = "Reset Robot")
public class ResetRobot extends OpMode {
    @Override
    public void init() {
        RobotContainer.ROBOT_POSE = null;
        RobotContainer.POSE_FROM_AUTO = false;
        stop();
    }

    @Override
    public void loop() {

    }
}
