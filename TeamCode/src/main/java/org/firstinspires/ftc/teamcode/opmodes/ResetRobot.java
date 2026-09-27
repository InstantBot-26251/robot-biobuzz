package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotContainer;

@TeleOp(name = "Reset Robot")
public class ResetRobot extends OpMode {
    @Override
    public void init() {
        RobotContainer.ROBOT_POSE = null;
        stop();
    }

    @Override
    public void loop() {

    }
}
