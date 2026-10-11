package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;

import org.firstinspires.ftc.teamcode.util.AutoFactory;

public class example extends AutoFactory {
    private final Pose path1 = poseFactory.of(85.6805, 109.4573, 270);

    @Override
    protected Pose setStartPose() {
        return poseFactory.of(85.861, 132.7149, 270);
    }

    private Path path1() {
        return Paths.line(startPose, path1).linear(startPose, path1);
    }

    @Override
    protected Command createAutoSequence() {
        return Commands.sequence(
                robot.followPath(path1())
        );
    }

}
