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

@Autonomous(name = "RedAlliancePreload")
public class RedPreloadAllianceAuto extends OpMode {
    private RobotContainer robot;
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose startPose = p.of(0, 0, 90);
    private final Pose driveForwardPose = p.of(0, 48, 90);

    private Path driveForwardPath() {
        return line(startPose, driveForwardPose).tangent();
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
                Commands.waitMillis(1000)
        ));
    }

    @Override
    public void loop() {
        robot.periodic();
    }
}