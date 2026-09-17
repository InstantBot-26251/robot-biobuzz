package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.test.RobotContainer;

@TeleOp(name = "Teleop")
public class DriverControlled extends OpMode {

   private RobotContainer robot;

   @Override
    public void init() {
       robot = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2);
   }

   @Override
    public void loop() {
       robot.periodic();
   }
}
