package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Commands;

import org.firstinspires.ftc.teamcode.RobotContainer;

@Autonomous(name = "Blue Alliance Straight Shoot-D,6")
public class BlueAlliance1StraightShoot extends OpMode {
    private RobotContainer robot;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(85.861, 132.7149, 270);
    private final Pose path1 = poseFactory.of(85.6805, 109.4573, 270);

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
