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
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(19.6947, 9.4014, 90);
    private final Pose point1 = poseFactory.of(17.1866, 47.6246, 93.7542);
    private final Pose point2 = poseFactory.of(24.7084, 93.838, 99.2445);

    public Path path1() {
        return line(start, point1).linear(start, point1);
    }

    public Path path2() {
        return line(point1, point2).linear(point1, point2);
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
                robot.followPath(path1()),
                Commands.waitMillis(1000),
                robot.followPath(path2())
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