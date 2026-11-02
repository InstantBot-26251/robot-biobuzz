package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
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

// Mirror of RedAlliance1StraightShoot, rotated 180 degrees around the field center
@Autonomous(name = "Blue Alliance Straight Shoot-D,6")
public class BlueAlliance1StraightShoot extends AutoFactory {
    private static final double FIELD_SIZE_IN = 144;

    @Override
    protected Pose setStartPose() {
        return mirror(56.9587, 9.447, 90);
    }

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

    private final Pose start = mirror(56.9587, 9.447, 90);
    private final Pose forward = mirror(56.9587, 15.447, 90);
    // Face the blue hive when shooting so the Limelight (front of robot) can see the tags
    private final Pose point1 = facingBlueHive(FIELD_SIZE_IN - 56.9587, FIELD_SIZE_IN - 22.7419);
    private final Pose point2 = mirror(70.6855, 47.484, 60.9389);
    private final Pose point3 = mirror(70.1692, 94.8227, 90.6249);
    private final Pose point4 = mirror(46.5804, 118.831, 134.495);
    private final Pose point5 = mirror(18.6337, 113.1839, -170.274);

    // Takes a red-side pose and returns the matching blue-side pose
    private Pose mirror(double redX, double redY, double redHeadingDeg) {
        return poseFactory.of(FIELD_SIZE_IN - redX, FIELD_SIZE_IN - redY, redHeadingDeg + 180);
    }

    private static Pose facingBlueHive(double x, double y) {
        return new Pose(x, y, Math.atan2(VisionConstants.BLUE_HIVE_Y_IN - y, VisionConstants.HIVE_CENTER_X_IN - x));
    }

    public Path driveForward() {
        return Paths.line(start, forward).constant(start);
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

    public Path path1() {
        return Paths.line(forward, point1).linear(forward, point1);
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
                robot.followPath(path1()),
                robot.aimAndShoot(AutoConstants.SHOOT_TIME),
                robot.followPath(park())
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
