package org.firstinspires.ftc.teamcode.shooter;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;

import org.firstinspires.ftc.teamcode.vision.Vision;

import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.*;

import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;

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

    public static Command autoAim(Shooter shooter, Vision vision) {
        return Commands.run(() -> aim(shooter, vision), shooter, vision);
    }

    public static Command aimShooter(Shooter shooter, Vision vision) {
        return Commands.runOnce(() -> aim(shooter, vision), shooter, vision);
    }

    public static Command shoot(Shooter shooter, Vision vision) {
        return Commands.sequence(Commands.runOnce(shooter::enableShooter, shooter),
                aimShooter(shooter, vision),
    public static Command shoot(Shooter shooter) {
        return Commands.sequence(Commands.runOnce(shooter::enableShooter, shooter),
                Commands.waitMillis(10),
                Commands.runOnce(shooter::startUpServos, shooter));
    }

    private static void aim(Shooter shooter, Vision vision) {
        if (!vision.hasTarget(TAG_LOST_HOLD_MS)) {
            shooter.setTargetVelocity(SHOOTER_VELOCITY);
            return;
        }

        double rpm = ShotCalculator.targetRpm(vision.getDistanceInches());
        if (Double.isNaN(rpm)) {
            shooter.setTargetVelocity(SHOOTER_VELOCITY);
            return;
        }

        shooter.setTargetVelocity(rpm);
        if (AUTO_SPIN_UP) {
            shooter.enableShooter();
        }
    }
}
