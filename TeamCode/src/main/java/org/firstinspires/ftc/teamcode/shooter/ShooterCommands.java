package org.firstinspires.ftc.teamcode.shooter;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;

public class ShooterCommands {

    public static Command shoot(Shooter shooter) {
        return new SequentialCommandGroup(
                new InstantCommand(shooter::spinUpFlywheel, shooter),
                new WaitCommand(Shooter.FLYWHEEL_SPIN_UP_MS),
                new InstantCommand(shooter::openDoor, shooter)
        );
    }

    public static Command increaseShotPower(Shooter shooter) {
        return new InstantCommand(shooter::increasePower, shooter);
    }

    public static Command decreaseShotPower(Shooter shooter) {
        return new InstantCommand(shooter::decreasePower, shooter);
    }

    public static Command stop(Shooter shooter) {
        return new InstantCommand(() -> {
            shooter.closeDoor();
            shooter.stopFlywheel();
        }, shooter);
    }

    public static Command resetShot(Shooter shooter) {
        return new InstantCommand(shooter::resetShotPower, shooter);
    }

    public static Command openDoor(Shooter shooter) {
        return new InstantCommand(shooter::openDoor, shooter);
    }

    public static Command closeDoor(Shooter shooter) {
        return new InstantCommand(shooter::closeDoor, shooter);
    }
}