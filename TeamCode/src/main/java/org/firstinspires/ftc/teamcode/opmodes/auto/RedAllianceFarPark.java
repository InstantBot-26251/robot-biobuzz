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
import org.firstinspires.ftc.teamcode.vision.VisionConstants;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name = "Red Alliance Far Park-C,1")
public class RedAllianceFarPark extends AutoFactory {

    @Override
    protected Pose setStartPose() {
        return poseFactory.of(56.9587, 9.447, 90);
    }

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56.9587, 9.447, 90);
    private final Pose point1 = new Pose(56.9587, 21.447, Math.atan2(
            VisionConstants.RED_HIVE_Y_IN - 21.447, VisionConstants.HIVE_CENTER_X_IN - 56.9587));

    private final Pose point2 = poseFactory.of(60.6855, 47.484, 60.9389);
    private final Pose point3 = poseFactory.of(60.1692, 94.8227, 90.6249);
    private final Pose point4 = poseFactory.of(46.5804, 100, 134.495);
    private final Pose point5 = poseFactory.of(16, 116, -170.274);

    public Path driveForward() {
        return Paths.line(start, point1).constant(start);
    }

    public Path park() {
        return roundedPath(CORNER_RADIUS_IN, point1, point2, point3, point4, point5);
    }

    private static final double CORNER_RADIUS_IN = 8;

    private Path roundedPath(double cornerIn, Pose... points) {
        List<Path> pieces = new ArrayList<>();
        Pose from = points[0];
        for (int i = 1; i < points.length - 1; i++) {
            Pose before = toward(points[i], points[i - 1], cornerIn);
            Pose after = toward(points[i], points[i + 1], cornerIn);
            pieces.add(Paths.line(from, before));
            pieces.add(Paths.curve(before, points[i], after));
            from = after;
        }
        pieces.add(Paths.line(from, points[points.length - 1]));
        return Paths.path(pieces.toArray(new Path[0])).tangent();
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
                robot.followPath(driveForward()),
                robot.aimAndShoot(AutoConstants.SHOOT_TIME),
                robot.followPath(park())
        );
    }
}
