package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class RobotContainer extends Robot {
    private final List<Subsystem> subsystems = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();


    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private GamepadEx gamepad1;
    private GamepadEx gamepad2;

    public RobotContainer(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {

        reset();

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = new GamepadEx(gamepad1);
        this.gamepad2 = new GamepadEx(gamepad2);

        subsystems.addAll(List.of());

        for(Subsystem s : subsystems) {
            register(s);
        }
    }

    @Override
    public void reset() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
    }

    public void periodic() {
        run();

        telemetry.addLine();
        telemetry.addData("Loop Time", timer.milliseconds());
        telemetry.update();
        timer.reset();

    }
}
