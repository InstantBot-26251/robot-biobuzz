package org.firstinspires.ftc.teamcode.defloweringStick;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;

public class DefloweringStickCommands {

    public static Command deflower(DefloweringStick defloweringStick) {
        return new InstantCommand(defloweringStick::deflower, defloweringStick);
    }

    public static Command restDeflowering (DefloweringStick defloweringStick) {
        return new InstantCommand(defloweringStick::deflower, defloweringStick);
    }
}
