package org.firstinspires.ftc.teamcode.shooter;


import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.vision.Vision;

import java.util.Locale;


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
        return Commands.run(() -> {
            if (!AUTO_AIM_ENABLED || !vision.hasTarget(TAG_LOST_HOLD_MS)) {
                shooter.setFlywheelPower(SHOOTER_POWER, AUTO_AIM_ENABLED ? "no tag, default" : "auto aim off");
                return;
            }

            double distance = vision.getDistanceInches();
            double power = ShotCalculator.motorPower(distance, shooter.getBatteryVoltage());
            if (Double.isNaN(power)) {
                shooter.setFlywheelPower(SHOOTER_POWER, "too close for launch angle");
                return;
            }

            shooter.setFlywheelPower(power, String.format(Locale.US, "auto @ %.0f in", distance));
            if (AUTO_SPIN_UP) {
                shooter.enableShooter();
            }
        }, shooter);
    }


    public static Command shoot(Shooter shooter) {
        return Commands.sequence(Commands.runOnce(shooter::enableShooter, shooter),
                Commands.waitMillis(SERVO_WAIT_TIME),
                Commands.runOnce(shooter::startUpServos, shooter));
    }
}
