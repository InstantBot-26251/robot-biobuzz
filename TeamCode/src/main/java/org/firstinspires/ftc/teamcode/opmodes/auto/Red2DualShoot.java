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

@Autonomous(name = "Red Alliance SGSP-C1")
public class Red2DualShoot extends AutoFactory {

    @Override
    protected Alliance getAlliance(){return Alliance.RED;}

    @Override
    protected Pose setStartPose() {
        return start;
    }

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 9.1657, 270);
    private final Pose path1 = poseFactory.of(56, 24.45, -90);
    private final Pose point2 = poseFactory.of(56, 9.1657, 180);
    private final Pose point3 = poseFactory.of(9, 8.97, -179.7615);
    private final Pose point4 = poseFactory.of(56, 24.45, 180);
    private final Pose point5 = poseFactory.of(56, 24.4468, 270);
    private final Pose point6 = poseFactory.of(24.45, 24.5795, 179.759);
    private final Pose point7 = poseFactory.of(24.45, 95.553, 90);
    private final Pose point8 = poseFactory.of(13.0815, 95.6501, 179.5104);

    public Path path1() {
        return Paths.line(start, path1).reverseTangent();
    }

    public Path path2() {
        return Paths.line(path1, point2).constant(point2);
    }

    public Path path3() {
        return Paths.line(point2, point3).tangent();
    }

    public Path path4() {
        return Paths.line(point3, point4).constant(point4);
    }

    public Path path6() {
        return Paths.line(point5, point6).tangent();
    }

    public Path path7() {
        return Paths.line(point6, point7).tangent();
    }

    public Path path8() {
        return Paths.line(point7, point8).tangent();
    }

    @Override
    protected Command createAutoSequence() {
        return Commands.sequence(
                robot.enableShooter(),
                robot.followPath(path1()),
                robot.aimAndShoot(),
                robot.followPath(path2()),
                robot.startIntake(),
                robot.followPath(path3()),
                robot.followPath(path4()),
                robot.stopIntake(),
                robot.turnTo(270),
                robot.aimAndShoot(),
                robot.followPath(path6()),
                robot.followPath(path7()),
                robot.followPath(path8())
        );
    }
}
