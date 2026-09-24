package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.commands.TeleopDriveCommand;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

import java.util.ArrayList;
import java.util.List;

public class RobotContainer extends Robot {
    private final List<SubsystemIF> subsystems = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();


    private final Chassis chassis;

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

        chassis = new Chassis(telemetry, hardwareMap);

        subsystems.addAll(List.of(chassis));

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

    public void autonomousInit() {
        for(SubsystemIF s : subsystems) {
            s.autonomousInit();
        }
    }
    public void teleopInit() {
        for(SubsystemIF s : subsystems) {
            s.teleopInit();
        }

        chassis.setDefaultCommand(new TeleopDriveCommand(
                chassis,
                () -> gamepad1.getLeftY(),
                () -> -gamepad1.getLeftX(),
                () -> -gamepad1.getRightX()));
        gamepad1.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(chassis::resetHeading);
    }

    public void periodic() {
        run();

        telemetry.addLine();
        telemetry.addData("Loop Time", timer.milliseconds());
        telemetry.update();
        timer.reset();

    }
}
