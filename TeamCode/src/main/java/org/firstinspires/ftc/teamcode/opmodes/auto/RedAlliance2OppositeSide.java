package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.api.Paths;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;

import org.firstinspires.ftc.teamcode.util.AutoFactory;

public class RedAlliance2OppositeSide extends AutoFactory {
    private final Pose start = poseFactory.of(32.176, 132.7149, 270);
    private final Pose path1 = poseFactory.of(32.176, 55.6728, 270);
    private final Pose point2 = poseFactory.of(32.1231, 26.3622, -90.1035);
    private final Pose point3 = poseFactory.of(56.639, 26.4209, -179.8629);

    public Path path1() {
        return Paths.line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.line(path1, point2).tangent();
    }

    public Path path3() {
        return Paths.line(point2, point3).reverseTangent();
    }

    @Override
    protected Pose setStartPose() {
        return poseFactory.of(85.861, 132.7149, 270);
    }

    @Override
    protected Command createAutoSequence() {
        return Commands.sequence(
                robot.followPath(path1())
        );
    }
}
