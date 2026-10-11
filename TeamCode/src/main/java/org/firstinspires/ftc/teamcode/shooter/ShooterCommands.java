package org.firstinspires.ftc.teamcode.shooter;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.WaitCommand;

public class ShooterCommands {
    public static Command enableShooter(Shooter shooter) {
        return new InstantCommand(shooter::enableShooter, shooter);
    }

    public static Command disableShooter(Shooter shooter) {
        return new InstantCommand(shooter::stopShooter, shooter);
    }

    public static Command shoot(Shooter shooter) {
       return new InstantCommand(shooter::enableShooter).andThen(new WaitCommand(1000))
               .andThen(new InstantCommand(shooter::startUpServos)).andThen(new WaitCommand(500))
               .andThen(new InstantCommand(shooter::stopShooter));
    }
}