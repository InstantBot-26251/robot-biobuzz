package org.firstinspires.ftc.teamcode.shooter;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;

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

    public static Command shoot(Shooter shooter) {
        return Commands.sequence(Commands.runOnce(shooter::enableShooter, shooter),
                Commands.waitMillis(10),
                Commands.runOnce(shooter::startUpServos, shooter));
    }
}
