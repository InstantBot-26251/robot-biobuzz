package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;

import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.AutoFactory;

@Autonomous(name = "Blue Alliance Straight Shoot-D,6")
public class BlueAlliance1StraightShoot extends AutoFactory {
    private final Pose point1 = poseFactory.of(85.6805, 109.4573, 270);

    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }

    @Override
    protected Pose setStartPose() {
        return poseFactory.of(85.861, 132.7149, 270);
    }

    private Path path1() {
        return Paths.line(startPose, point1).linear(startPose, point1);
    }

    @Override
    protected Command createAutoSequence() {
        return Commands.sequence(
                robot.enableShooter(),
                robot.followPath(path1()),
                robot.shootFor(AutoConstants.SHOOT_TIME)
        );
    }
}