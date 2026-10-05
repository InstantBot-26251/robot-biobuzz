package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandScheduler;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.Subsystem;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.intake.IntakeCommands;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.ChassisCommands;
import org.firstinspires.ftc.teamcode.shooter.Shooter;
import org.firstinspires.ftc.teamcode.shooter.ShooterCommands;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;


import java.util.ArrayList;
import java.util.List;

public class RobotContainer extends Robot {
    public static Pose ROBOT_POSE = null;
    private final List<SubsystemIF> subsystems = new ArrayList<>();
    private final ElapsedTime timer = new ElapsedTime();
    private final List<LynxModule> hubs;
    private final Chassis chassis;
    private final Shooter shooter;
    private final Intake intake;

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


        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        chassis = new Chassis(telemetry, hardwareMap);
        shooter = new Shooter(telemetry, hardwareMap);
        intake = new Intake(telemetry, hardwareMap);

        subsystems.addAll(List.of(chassis, intake, shooter));

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

        chassis.setDefaultCommand(ChassisCommands.createTeleopDriveCommand(
                chassis,
                () -> gamepad1.getLeftY(),
                () -> -gamepad1.getLeftX(),
                () -> -gamepad1.getRightX()));

        gamepad1.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(chassis::resetHeading);

        gamepad2.getGamepadButton(GamepadKeys.Button.A).whenPressed(ShooterCommands.enableShooter(shooter));
        gamepad2.getGamepadButton(GamepadKeys.Button.B).whenPressed(ShooterCommands.disableShooter(shooter));
        gamepad2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(ShooterCommands.shoot(shooter));
        gamepad2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(IntakeCommands.intake(intake))
                .whenReleased(IntakeCommands.stopIntake(intake));
        gamepad2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(IntakeCommands.outtake(intake))
                .whenReleased(IntakeCommands.stopIntake(intake));
    }

    public void stop() {
        ROBOT_POSE = chassis.getPose();
    }

    public void setPose(Pose pose) {
        chassis.setPose(pose);
    }

    public Command followPath(Path path) {
        return ChassisCommands.createFollowPathCommand(chassis, path);
    }

    public Command followPath(Path path, boolean holdEnd) {
        return ChassisCommands.createFollowPathCommand(chassis, path, holdEnd);
    }

    public void periodic() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        run();

        telemetry.addLine();
        telemetry.addData("Loop Time", timer.milliseconds());
        telemetry.update();
        timer.reset();

    }
}