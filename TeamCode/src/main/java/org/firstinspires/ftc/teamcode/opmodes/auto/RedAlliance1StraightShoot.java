package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;

import org.firstinspires.ftc.teamcode.opmodes.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoFactory;

@Autonomous(name = "Red Alliance Straight Shoot-C,1")
public class RedAlliance1StraightShoot extends AutoFactory {
    private final Pose point1 = poseFactory.of(55.8195, -23, 90);

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected Pose setStartPose() {
        return poseFactory.of(55.842, -9.0242, 90);
    }

    private Path path1() {
        return Paths.line(startPose, point1).linear(startPose, point1);
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
