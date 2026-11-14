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

@Autonomous(name = "Blue Alliance SP-C1")
public class Blue1ShootPark extends AutoFactory {

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected Pose setStartPose() {
        return start;
    }

    private static final double CORNER_RADIUS_IN = 8;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 135.1856, 90);
    private final Pose shootSpot = poseFactory.of(56, 117.3655, 90);
    private final Pose point2 = poseFactory.of(33, 117.37, 90);
    private final Pose point3 = poseFactory.of(33, 29, 90);
    private final Pose point4 = poseFactory.of(15, 29, 0);

    public Path path1() {
        return Paths.line(start, shootSpot).constant(start);
    }

    public Path path2() {
        return Paths.line(shootSpot, point2).constant(point2);
    }

    public Path park() {
        Pose beforeCorner = toward(point3, point2, CORNER_RADIUS_IN);
        Pose afterCorner = toward(point3, point4, CORNER_RADIUS_IN);
        return Paths.path(
                Paths.line(point2, beforeCorner),
                Paths.curve(beforeCorner, point3, afterCorner),
                Paths.line(afterCorner, point4)
        ).tangent();
    }

    private Pose toward(Pose from, Pose to, double distance) {
        double length = Math.hypot(to.x() - from.x(), to.y() - from.y());
        double d = Math.min(distance, length * 0.4);
        return new Pose(from.x() + (to.x() - from.x()) * d / length,
                from.y() + (to.y() - from.y()) * d / length, 0);
    }

    @Override
    protected Command createAutoSequence() {
        return Commands.sequence(
                robot.enableShooter(),
                robot.followPath(path1()),
                robot.aimAndShoot(),
                robot.followPath(path2()),
                robot.followPath(park())
        );
    }
}
