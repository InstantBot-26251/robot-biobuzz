package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Commands;

import org.firstinspires.ftc.teamcode.RobotContainer;

@Autonomous(name = "Red Alliance Straight Shoot-C,1")
public class RedAlliance1StraightShoot {
    private RobotContainer robot;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(55.842, 9.0242, 90);
    private final Pose path1 = poseFactory.of(55.8195, 32.2098, 90);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    @Override
    public void init() {
        robot = new RobotContainer(hardwareMap, telemetry, gamepad1, gamepad2);
        robot.autonomousInit();

        robot.setPose(start);
    }

    @Override
    public void start() {
        CommandScheduler.getInstance().schedule(Commands.sequence(
                robot.followPath(path1())
        ));
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
