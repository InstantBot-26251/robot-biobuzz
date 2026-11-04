package org.firstinspires.ftc.teamcode.shooter;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;

import java.util.function.BooleanSupplier;

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

    public static Command holdFixedRpm(Shooter shooter) {
        return Commands.runOnce(shooter::holdFixedRpm, shooter);
    }

    public static Command releaseFixedRpm(Shooter shooter) {
        return Commands.runOnce(shooter::releaseFixedRpm, shooter);
    }

    public static Command releaseShoot(Shooter shooter, BooleanSupplier keepSpinning) {
        return Commands.runOnce(() -> {
            if (keepSpinning.getAsBoolean()) {
                shooter.stopServos();
            } else {
                shooter.stopShooter();
            }
        }, shooter);
    }

    public static Command trimUp(Shooter shooter) {
        return Commands.runOnce(() -> {
            if (shooter.isFixedRpmHeld()) {
                shooter.adjustFixedRpm(FIXED_RPM_STEP);
            } else {
                shooter.adjustTrim(MANUAL_TRIM_STEP_RPM);
            }
        });
    }

    public static Command trimDown(Shooter shooter) {
        return Commands.runOnce(() -> {
            if (shooter.isFixedRpmHeld()) {
                shooter.adjustFixedRpm(-FIXED_RPM_STEP);
            } else {
                shooter.adjustTrim(-MANUAL_TRIM_STEP_RPM);
            }
        });
    }

    public static Command resetTrim(Shooter shooter) {
        return Commands.runOnce(shooter::resetTrim);
    }
}
