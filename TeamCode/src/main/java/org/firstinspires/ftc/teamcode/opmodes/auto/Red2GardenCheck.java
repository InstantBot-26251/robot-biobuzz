package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;

import org.firstinspires.ftc.teamcode.opmodes.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoFactory;

@Autonomous(name = "Red Alliance SGCSP-C1")
public class Red2GardenCheck extends AutoFactory {

    @Override
    protected Alliance getAlliance(){return Alliance.RED;}

    @Override
    protected Pose setStartPose() {
        return start;
    }

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 9.1657, 270);
    private final Pose path1 = poseFactory.of(56, 31, -90);
    private final Pose point2 = poseFactory.of(11.0739, 32.8133, -103.4586);
    private final Pose point2Control1 = poseFactory.of(46.1089, 34.127, 0);
    private final Pose point2Control2 = poseFactory.of(17.9416, 49.514, 0);
    private final Pose point2Control3 = poseFactory.of(13.4907, 42.9121, 0);
    private final Pose point3 = poseFactory.of(9.0014, 22.1478, -100.9965);
    private final Pose point4 = poseFactory.of(8.8721, 9.2283, -90.5737);
    private final Pose point5 = poseFactory.of(25.4182, 26.001, -168.0389);
    private final Pose point5Control1 = poseFactory.of(7.9187, 22.2938, 0);
    private final Pose point6 = poseFactory.of(56.2114, 33.7722, -165.8362);
    private final Pose point7 = poseFactory.of(56.1791, 30.6637, -90.5968);
    private final Pose point8 = poseFactory.of(56.4705, 125.2453, -90.1765);
    private final Pose point9 = poseFactory.of(56.4705, 117.0532, 90);
    private final Pose point10 = poseFactory.of(7.0264, 117.2151, 179.8124);

    public Path path1() {
        return Paths.line(start, path1).reverseTangent();
    }

    public Path path2() {
        return Paths.curve(path1, point2Control1, point2Control2, point2Control3, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).tangent();
    }

    public Path path4() {
        return Paths.line(point3, point4).tangent();
    }

    public Path path5() {
        return Paths.curve(point4, point5Control1, point5).reverseTangent();
    }

    public Path path6() {
        return Paths.line(point5, point6).reverseTangent();
    }

    public Path path7() {
        return Paths.line(point6, point7).tangent();
    }

    public Path path8() {
        return Paths.line(point7, point8).reverseTangent();
    }

    public Path path9() {
        return Paths.line(point8, point9).reverseTangent();
    }

    public Path path10() {
        return Paths.line(point9, point10).tangent();
    }

    @Override
    protected Command createAutoSequence() {
        return Commands.sequence(
                robot.enableShooter(),
                robot.followPath(path1()),
                robot.aimAndShoot(),
                robot.followPath(path2(), false),
                robot.followPath(path3(), false),
                robot.followPath(path4(), false),
                robot.followPath(path5(), false),
                robot.followPath(path6(), false),
                robot.followPath(path7(), false),
                robot.followPath(path8(), false),
                robot.followPath(path9(), false),
                robot.followPath(path10())
        );
    }
}
