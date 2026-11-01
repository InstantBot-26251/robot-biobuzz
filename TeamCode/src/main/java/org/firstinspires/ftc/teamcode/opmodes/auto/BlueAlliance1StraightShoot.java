package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;

import org.firstinspires.ftc.teamcode.opmodes.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoFactory;

@Autonomous(name = "Blue Alliance Straight Shoot-D,6")
public class BlueAlliance1StraightShoot extends AutoFactory {
    private final Pose start = poseFactory.of(85.7319, 132.7149, 90);
    private final Pose path1 = poseFactory.of(85.6805, 119.4573, 89.7778);
    private final Pose point2 = poseFactory.of(73.3315, 95.21, -116.9894);
    private final Pose point3 = poseFactory.of(73.5897, 46.5482, -89.6961);
    private final Pose point4 = poseFactory.of(80.818, 23.8307, -72.3498);
    private final Pose point5 = poseFactory.of(94.2096, 23.1853, -2.7591);
    private final Pose point6 = poseFactory.of(126.9628, 23.2176, 0.0564);

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected Pose setStartPose() {
        return poseFactory.of(85.861, -132.7149, -90);
    }

    public Path path1() {
        return Paths.line(start, path1).reverseTangent();
    }

    public Path path2() {
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).tangent();
    }

    public Path path4() {
        return Paths.line(point3, point4).tangent();
    }

    public Path path5() {
        return Paths.line(point4, point5).tangent();
    }

    public Path path6() {
        return Paths.line(point5, point6).tangent();
    }

    @Override
    protected Command createAutoSequence() {
        return Commands.sequence(
                robot.enableShooter(),
                robot.followPath(path1(), true),
                robot.aimAndShoot(AutoConstants.SHOOT_TIME),
                robot.followPath(path2(), false),
                robot.followPath(path3(), false),
                robot.followPath(path4(), false),
                robot.followPath(path5(), false),
                robot.followPath(path6(), true)
        );
    }
}
