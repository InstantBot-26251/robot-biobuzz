package org.firstinspires.ftc.teamcode.shooter;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;

import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.MANUAL_TRIM_STEP_RPM;

public class ShooterCommands {
    public static Command enableShooter(Shooter shooter) {
        return Commands.runOnce(shooter::enableShooter, shooter);
    }

    public static Command disableShooter(Shooter shooter) {
        return Commands.runOnce(shooter::stopShooter, shooter);
    }

    public static Command disableShooterServo(Shooter shooter) {
        return Commands.runOnce(shooter::stopServos, shooter);
    }

    public static Command shoot(Shooter shooter) {
        return Commands.runOnce(() -> {
            if (shooter.getMode() == Shooter.Mode.OFF) shooter.enableShooter();
            shooter.startUpServos();
        }, shooter);
    }

    public static Command trimUp(Shooter shooter) {
        return Commands.runOnce(() -> shooter.adjustTrim(MANUAL_TRIM_STEP_RPM));
    }

    public static Command trimDown(Shooter shooter) {
        return Commands.runOnce(() -> shooter.adjustTrim(-MANUAL_TRIM_STEP_RPM));
    }

    public static Command resetTrim(Shooter shooter) {
        return Commands.runOnce(shooter::resetTrim);
    }
}
