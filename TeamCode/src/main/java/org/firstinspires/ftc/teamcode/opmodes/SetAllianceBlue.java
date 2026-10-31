package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Set Alliance: BLUE", group = "Alliance")
public class SetAllianceBlue extends OpMode {
    @Override
    public void init() {
        AllianceStore.save(Alliance.BLUE);
        telemetry.addLine("Alliance set to BLUE and saved.");
        telemetry.addLine("Limelight will only track blue hive tags (38-45).");
        telemetry.addLine("You can press STOP now.");
        telemetry.update();
    }

    @Override
    public void loop() {
    }
}
