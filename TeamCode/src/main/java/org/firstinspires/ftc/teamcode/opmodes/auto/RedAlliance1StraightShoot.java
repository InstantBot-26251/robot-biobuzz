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

@Autonomous(name = "Red Alliance Straight Shoot-C,1")
public class RedAlliance1StraightShoot extends AutoFactory {

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
    private final Pose point1 = poseFactory.of(56.9361, 22.7419, -89.9027);
    private final Pose point2 = poseFactory.of(70.6855, 47.484, 60.9389);
    private final Pose point3 = poseFactory.of(70.1692, 94.8227, 90.6249);
    private final Pose point4 = poseFactory.of(45.1606, 130.9641, 124.6819);
    private final Pose point5 = poseFactory.of(13.6337, 113.1839, -150.5782);

    public Path path1() {
        return Paths.line(start, point1).reverseTangent();
    }

    public Path path2() {
        return Paths.line(point1, point2).tangent();
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


    @Override
    protected Command createAutoSequence() {
        return Commands.sequence(
                robot.enableShooter(),
                robot.followPath(path1()),
                robot.aimAndShoot(AutoConstants.SHOOT_TIME)
        );
    }
}
