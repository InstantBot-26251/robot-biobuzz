package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Set Alliance: RED", group = "Alliance")
public class SetAllianceRed extends OpMode {
    @Override
    public void init() {
        AllianceStore.save(Alliance.RED);
        telemetry.addLine("Alliance set to RED and saved.");
        telemetry.addLine("Limelight will only track red hive tags (30-37).");
        telemetry.addLine("You can press STOP now.");
        telemetry.update();
    }

    @Override
    public void loop() {
    }
}
