package org.firstinspires.ftc.teamcode.shooter;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.InstantCommand;
import static org.firstinspires.ftc.teamcode.shooter.ShooterConstants.*;



public class ShooterCommands {
    public static Command enableShooter(Shooter shooter) {
        return new InstantCommand(shooter::enableShooter, shooter);
    }

    public static Command disableShooter(Shooter shooter) {
        return new InstantCommand(shooter::stopShooter, shooter);
    }

    public static Command shoot(Shooter shooter) {
      return Commands.sequence(Commands.runOnce(shooter::enableShooter),
               (Commands.waitMillis(SERVO_WAIT_TIME)),
               (Commands.runOnce(shooter::startUpServos)),
               (Commands.waitMillis(POST_CYCLE_WAIT_TIME)),
               (Commands.runOnce(shooter::stopShooter)));
    }
}