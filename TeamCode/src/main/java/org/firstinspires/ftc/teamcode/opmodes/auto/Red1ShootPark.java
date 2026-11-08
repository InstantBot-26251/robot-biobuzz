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

@Autonomous(name = "Red Alliance SP-C1")
public class Red1ShootPark extends AutoFactory {

    @Override
    protected Alliance getAlliance(){return Alliance.RED;}

    @Override
    protected Pose setStartPose() {
        return start;
    }

    private static final double CORNER_RADIUS_IN = 20;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 6.3162, 270);
    private final Pose path1 = poseFactory.of(56, 31.443, 270);
    private final Pose point2 = poseFactory.of(59, 45.5261, 90.4146);
    private final Pose point2Control1 = poseFactory.of(54.9388, 36.2705, 0);
    private final Pose point2Control2 = poseFactory.of(59.0584, 37.4536, 0);
    private final Pose point3 = poseFactory.of(59, 121.8165, 90);
    private final Pose point4 = poseFactory.of(12.531, 121.82, 179.9957);

    public Path path1() {
        return Paths.line(start, path1).reverseTangent();
    }

    public Path park() {
        Pose beforeCorner = toward(point3, point2, CORNER_RADIUS_IN);
        Pose afterCorner = toward(point3, point4, CORNER_RADIUS_IN);
        return Paths.path(
                Paths.curve(path1, point2Control1, point2Control2, point2),
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
                robot.followPath(park())
        );
    }
}
