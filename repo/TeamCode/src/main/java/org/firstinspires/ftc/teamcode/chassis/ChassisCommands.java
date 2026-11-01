package org.firstinspires.ftc.teamcode.chassis;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.FunctionalCommand;

import org.firstinspires.ftc.teamcode.vision.Limelight;
import org.firstinspires.ftc.teamcode.vision.TargetMath;

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
        return new FunctionalCommand(
                () -> {
                    timer.reset();
                    chassis.setHoldEnd(holdEnd);
                    chassis.followPath(path);
                },
                () -> {},
                interrupted -> {
                    boolean timedOut = timer.milliseconds() > PATH_TIMEOUT_MS;
                    if ((interrupted || timedOut) && chassis.isFollowingPath()) {
                        if (holdEnd) {
                            chassis.holdCurrentPose();
                        } else {
                            chassis.stopFollower();
                        }
                    }
                },
                () -> chassis.isPathFinished()
                        || chassis.hasLocalizationFault()
                        || timer.milliseconds() > PATH_TIMEOUT_MS,
                chassis
        );
    }

    public static Command createAimCommand(Chassis chassis, Limelight limelight) {
        ElapsedTime timer = new ElapsedTime();
        ElapsedTime settled = new ElapsedTime();
        double[] holding = new double[3];
        boolean[] state = new boolean[2];

        return new FunctionalCommand(
                () -> {
                    timer.reset();
                    settled.reset();
                    state[0] = false;
                    state[1] = false;
                    Pose pose = chassis.getPose();
                    holding[0] = pose.x();
                    holding[1] = pose.y();
                },
                () -> {
                    if (!limelight.hasTarget()) return;
                    double target = limelight.getAimFieldHeading(chassis.getPose());
                    if (!state[0] || Math.abs(TargetMath.normalizeAngle(target - holding[2])) > Math.toRadians(0.5)) {
                        holding[2] = target;
                        state[0] = true;
                        chassis.holdPose(new Pose(holding[0], holding[1], target));
                    }
                    double error = TargetMath.normalizeAngle(target - chassis.getPose().heading());
                    boolean inside = Math.abs(error) < Math.toRadians(AIM_TOLERANCE_DEG)
                            && Math.abs(chassis.getAngularVelocity()) < Math.toRadians(30);
                    if (inside && !state[1]) settled.reset();
                    state[1] = inside;
                },
                interrupted -> {},
                () -> timer.milliseconds() > AIM_TIMEOUT_MS
                        || (!limelight.hasTarget() && timer.milliseconds() > 250)
                        || (state[1] && settled.milliseconds() > AIM_SETTLE_MS),
                chassis
        );
    }
}
