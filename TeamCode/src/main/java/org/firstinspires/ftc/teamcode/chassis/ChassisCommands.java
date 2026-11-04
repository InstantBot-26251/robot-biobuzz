package org.firstinspires.ftc.teamcode.chassis;

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
}
