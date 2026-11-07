package org.firstinspires.ftc.teamcode.chassis;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;
import org.firstinspires.ftc.teamcode.vision.Limelight;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import static org.firstinspires.ftc.teamcode.chassis.DriveConstants.*;

public class ChassisCommands {
    public static Command createTeleopDriveCommand(Chassis chassis, Limelight limelight,
                                                   DoubleSupplier forward, DoubleSupplier left, DoubleSupplier turn,
                                                   BooleanSupplier aim) {
        return chassis.runEnd(
                () -> {
                    if (aim.getAsBoolean() && limelight.hasTarget()) {
                        chassis.setDrivePowersFacing(forward.getAsDouble(), left.getAsDouble(),
                                limelight.getAimFieldHeading(chassis.getPose()));
                    } else {
                        chassis.setDrivePowers(forward.getAsDouble(), left.getAsDouble(), turn.getAsDouble());
                    }
                },
                chassis::stopFollower
        );
    }

    public static Command createFollowPathCommand(Chassis chassis, Path path) {
        return createFollowPathCommand(chassis, path, true);
    }

    public static Command createFollowPathCommand(Chassis chassis, Path path, boolean holdEnd) {
        ElapsedTime timer = new ElapsedTime();
        double timeoutMs = Math.max(PATH_TIMEOUT_MS, 1500 + 1000 * pathLength(path) / PATH_TIMEOUT_MIN_SPEED);
        return new FunctionalCommand(
                () -> {
                    timer.reset();
                    chassis.setHoldEnd(holdEnd);
                    chassis.followPath(path);
                },
                () -> {},
                interrupted -> {
                    boolean timedOut = timer.milliseconds() > timeoutMs;
                    if ((interrupted || timedOut) && chassis.isFollowingPath()) {
                        if (holdEnd) {
                            chassis.holdCurrentPose();
                        } else {
                            chassis.stopFollower();
                        }
                    }
                    String result = chassis.hasLocalizationFault() ? "STOPPED by runaway guard"
                            : timedOut ? String.format(java.util.Locale.US, "TIMED OUT (limit %.1f s)", timeoutMs / 1000)
                            : interrupted ? "interrupted" : "reached end";
                    chassis.reportPathEnd(result, path, timer.seconds());
                },
                () -> chassis.isPathFinished()
                        || chassis.hasLocalizationFault()
                        || timer.milliseconds() > timeoutMs,
                chassis
        );
    }

    public static double pathLength(Path path) {
        double length = 0;
        Pose previous = path.get(0);
        for (int i = 1; i <= 200; i++) {
            Pose next = path.get(i / 200.0);
            length += Math.hypot(next.x() - previous.x(), next.y() - previous.y());
            previous = next;
        }
        return length;
    }
}
