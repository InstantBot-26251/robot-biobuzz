package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.command.SubsystemBase;

public abstract class SubsystemIF extends SubsystemBase {
    public abstract void autonomousInit();
    public abstract void teleopInit();
}
