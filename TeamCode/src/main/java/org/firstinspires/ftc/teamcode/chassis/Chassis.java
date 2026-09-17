package org.firstinspires.ftc.teamcode.chassis;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Chassis extends SubsystemBase {
    private Telemetry telemetry;
    private Follower follower;

    public Chassis(Telemetry telemetry, HardwareMap hardwareMap) {
        this.telemetry = telemetry;
        follower = Constants.create(hardwareMap);
    }

}
