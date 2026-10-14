package org.firstinspires.ftc.teamcode.shooter;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;

import org.firstinspires.ftc.teamcode.vision.Vision;

import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;
import static org.firstinspires.ftc.teamcode.vision.VisionConstants.*;

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
        return Commands.runEnd(
                () -> aim(shooter, vision),
                () -> shooter.setMotorPower(SHOOTER_POWER),
                shooter, vision);
    }

    public static Command shoot(Shooter shooter, Vision vision) {
        return Commands.sequence(Commands.runOnce(shooter::enableShooter, shooter),
                Commands.runOnce(() -> aim(shooter, vision), shooter, vision),
                Commands.waitMillis(SERVO_WAIT_TIME),
                Commands.runOnce(shooter::startUpServos, shooter));
    }

    private static void aim(Shooter shooter, Vision vision) {
        if (!vision.hasTarget(TAG_LOST_HOLD_MS)) {
            shooter.setMotorPower(SHOOTER_POWER);
            return;
        }

        double power = ShotCalculator.motorPower(vision.getDistanceInches());
        if (Double.isNaN(power)) {
            shooter.setMotorPower(SHOOTER_POWER);
            return;
        }

        shooter.setMotorPower(power);
        if (AUTO_SPIN_UP) {
            shooter.enableShooter();
        }
    }
}
