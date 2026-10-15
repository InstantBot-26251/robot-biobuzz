package org.firstinspires.ftc.teamcode.depollinator;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.Commands;
import com.seattlesolvers.solverslib.command.InstantCommand;

public class DepollinatorCommands {
    public static Command depollinate(Depollinator depollinator) {
        return Commands.runOnce(depollinator::depollinate, depollinator);
    }

    public static Command restDepollinator(Depollinator depollinator) {
        return Commands.runOnce(depollinator::restDepollinator, depollinator);
    }
}
