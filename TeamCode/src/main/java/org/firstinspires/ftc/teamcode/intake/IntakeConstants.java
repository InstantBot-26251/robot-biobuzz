package org.firstinspires.ftc.teamcode.intake;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class IntakeConstants {
    public static double MAX_ACHIEVABLE_RPM = 6000;
    public static double TICKS_PER_REVOLUTION = 28;

    public static double INTAKE_VELOCITY = 4500;
    public static double OUTTAKE_VELOCITY = -INTAKE_VELOCITY;
    public static double STOP_INTAKE_VELOCITY = 0;

    public static double INTAKE_P = 1;
    public static double INTAKE_I = 0;
    public static double INTAKE_D = 0;
    public static double INTAKE_F = 0;

    public static double toTicksPerSec(double rpm) {
        return (rpm / 60) * TICKS_PER_REVOLUTION;
    }
}
