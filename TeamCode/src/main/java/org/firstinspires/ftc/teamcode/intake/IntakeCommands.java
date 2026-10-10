package org.firstinspires.ftc.teamcode.intake;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;

public class IntakeCommands {
    public static Command intake(Intake intake) {
        return new InstantCommand(intake::intake, intake);
    }

    public static Command outtake(Intake intake) {
        return new InstantCommand(intake::outtake, intake);
    }

    public static Command noIntake(Intake intake) {
        return new InstantCommand(intake::noIntake, intake);
    }

    public static Command stopIntake(Intake intake) {
        return new InstantCommand(intake::stop, intake);
    }
}