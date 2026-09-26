package org.firstinspires.ftc.teamcode.opmodes;


import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Commands;

import org.firstinspires.ftc.teamcode.RobotContainer;
import org.firstinspires.ftc.teamcode.chassis.ChassisCommands;


@Autonomous(name = "RedAlliancePreload")
public class RedPreloadAllianceAuto extends OpMode {
    private RobotContainer robot;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(32.22493224932249, 14.902439024390237, 90);
    private final Pose driveForwardPose = p.of(32.16176948564556, 41.512283907156146, 88);
    private final Pose finalParkPose = p.of(30.674796747967477, 94.48915989159892, 88);

    private Path driveForwardPath() {
        return line(startPose, driveForwardPose).linear(startPose, driveForwardPose);
    }

    private Path finalPath() {
        return line(driveForwardPose, finalParkPose).linear(driveForwardPose, finalParkPose);
    }

    @Override
    public void init() {
        robot = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.autonomousInit();

        robot.setPose(startPose);
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(Commands.sequence(
            robot.followPath(driveForwardPath()),
            Commands.waitMillis(1000),
            robot.followPath(finalPath())
        ));
    }

    @Override
    public void loop() {
        robot.periodic();

    }
}
